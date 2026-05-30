# SPARE-PIM Simulator Implementation Plan

This document tracks the implementation plan for the SPARE-PIM simulator built
on top of Ramulator 2.1 / Ramulator2. The goal is to model the Elemax-based
single-pass SDPA flow, immediate dynamic sparsity, BGMU/VOC-driven SPM latency,
and ESDM data placement effects.

## 0. Implementation Principles

- Keep the existing Ramulator component structure intact.
- Keep regular HBM3 behavior and SPARE-PIM experiments separated through
  explicit frontend/controller/config choices.
- Do not require a new timing preset for SPARE-PIM. Paper-specific timing values
  are applied as YAML overrides where possible.
- Model timing-visible behavior first: command scheduling, metadata latency,
  sparsity skip, dynamic SPM latency, and page-hit behavior.
- Add the FP16 functional dataflow model as a separate layer so timing
  experiments can still run without tensor-value simulation.

## 1. DRAM/HBM3 Command And Timing Layer

### Goal

Extend the HBM3 model with SPARE-PIM RFU-style commands while preserving the
standard Ramulator timing path.

### Implemented Work

- Added SPARE-PIM commands in `src/dram/impl/HBM3.cpp`:
  - `MAC`: all-bank Q x K^T MAC command
  - `EMUL`: all-bank element-wise multiply command
  - `MOV`: VB/bank movement command with optional ReLU semantics
  - `SPM`: all-bank sparse long-data command
  - `ACC`: single-bank transfer to the channel-level accumulator
- Added request-name translation:
  - `mac -> MAC`
  - `emul -> EMUL`
  - `mov -> MOV`
  - `spm -> SPM`
  - `acc -> ACC`
  - `spare-pim -> controller-expanded token request`
- Added `nSPM` as the timing field for the sparse kernel.
- Added a DRAM notification hook:
  - key: `sparepim_spm_latency`
  - value: controller-computed dynamic `SPM` latency

### Timing Model

- `MAC`: `tCCDL`
- `EMUL`: `tCCDL`
- `MOV`: `tCCDS`
- `ACC`: `tCCDL + 1`
- `SPM`: dynamic `nSPM`, updated by the controller before each sparse kernel

### Status

Implemented and build-tested.

## 2. SPARE-PIM Controller

### Goal

Model the interaction between the memory controller and SPARE-PIM hardware. The
controller must expand a token-level request into the ESEF command flow and
handle BGMU READ, VOC aggregation, zero-VOC skipping, and dynamic SPM latency.

### Implemented Files

- `src/dram_controller/impl/sparepim_controller.cpp`
- `src/dram_controller/CMakeLists.txt`

### ESEF Expansion

The controller expands one `spare-pim` request as follows:

1. QK stage
   - Issue repeated `MAC` commands.
   - Default command count is derived from `ceil(d_head / fsu_width)`.
   - A per-token override can be supplied through the request scratchpad.
2. Elemax stage
   - Issue `MOV`.
   - Model ReLU bitmask generation as occurring at `MOV` time.
   - Issue `EMUL`.
   - Model DSSE VDI/VOC generation as fully overlapped with `EMUL`.
3. BGMU READ stage
   - Read BGMU VOC register groups from the token metadata payload.
   - Each BGMU register group contains one VOC per bank/BPU.
   - Update `max_count` while scanning the VOC values.
   - Model BGMU reads as a controller-local delay.
   - Latency:

```text
L_BGMU = nCL + (N_BG - 1) * nCCDS + nBL
```

4. Sparse PV stage
   - Use the controller-computed `max_count`.
   - If `max_count == 0`, skip sparse `ACT-SPM-PRE`.
   - If `max_count > 0`, update `nSPM` through the DRAM notify hook and issue
     sparse PV commands.
5. Accumulation stage
   - Issue `ACC` commands for partial-sum transfer and accumulation.
   - `ACC` uses the HBM3 timing constraint `tCCDL + 1`.
6. Complete the original token request callback.

### Request Scratchpad ABI

For aggregate token formats, the `SPAREPIM` controller still accepts compact
metadata through `Request::scratchpad`:

```text
scratchpad[0] = max_voc
scratchpad[1] = total_voc
scratchpad[2] = per-token MAC command override, 0 means use default
scratchpad[3] = per-token ACC command override, 0 means use default
```

For `VOC` trace lines, the frontend attaches the full BPU VOC vector as the
request payload. The controller scans this vector in BGMU groups, computes
`max_count`, and then configures `SPM(max_count)`.

### Important Config Parameters

- `queue_size`: controller input queue size
- `spm_cycles_per_voc`: dynamic SPM cycles per valid output count
- `spm_fixed_cycles`: optional fixed term added to SPM latency
- `mac_commands_per_token`: default QK-stage MAC count
- `acc_commands_per_token`: default ACC count
- `bgmu_read_as_delay`: model BGMU READ as local controller delay
- `enable_act_spm_pre`: issue ACT/PRE around sparse PV
- `enable_esdm`: enable ESDM-oriented address generation
- `enable_qk_act_pre`: issue ACT/PRE around QK rows
- `esdm_columns_per_row`: number of consecutive columns used per ESDM row

### Statistics

- `sparepim_tokens`
- `sparepim_mac_cmds`
- `sparepim_mov_cmds`
- `sparepim_emul_cmds`
- `sparepim_spm_cmds`
- `sparepim_acc_cmds`
- `sparepim_bgmu_reads`
- `sparepim_bgmu_voc_values`
- `sparepim_bgmu_max_updates`
- `sparepim_bgmu_read_cycles`
- `sparepim_spm_cycles`
- `sparepim_skipped_spm`
- `sparepim_total_voc`
- `sparepim_max_voc_sum`

### Status

Implemented for the performance/timing simulator.

## 3. Frontend And Workload Input

### Goal

Provide a token-level frontend for inference timing experiments. The frontend
must feed VOC metadata into the `SPAREPIM` controller and wait for full command
completion.

### Implemented Files

- `src/frontend/impl/sparepim/sparepim_trace.cpp`
- `src/frontend/CMakeLists.txt`

### Supported Input Formats

```text
VOC voc0,voc1,... [addr] [mac_cmds] [acc_cmds]
TOKEN max_voc total_voc [addr] [mac_cmds] [acc_cmds]
TOKEN layer head batch q_idx voc0,voc1,... [mac_cmds] [acc_cmds]
max_voc total_voc [addr] [mac_cmds] [acc_cmds]
```

The frontend can also generate a fixed synthetic trace when no path is provided:

- `num_tokens`
- `fixed_max_voc`
- `fixed_total_voc`

### Validation

- Clamp negative VOC entries to zero.
- Validate the VOC list length against the BPU count per pseudochannel when
  `validate_voc_count=true`.
- Reject `max_voc > max_voc_per_bpu`.
- Reject `total_voc > expected_voc_count * max_voc_per_bpu`.
- Reject `max_voc > total_voc` when `total_voc > 0`.

### Model Parameters

Supported model/workload parameters include:

- `num_layers`
- `num_heads`
- `num_kv_heads`
- `seq_len`
- `d_head`
- `batch_size`
- `fsu_width`
- `max_voc_per_bpu`
- `expected_voc_count`

### Status

Implemented and validated with `example_sparepim.trace`.

## 4. ESDM Address Mapping And Page-Hit Model

### Goal

Reflect the paper's K/V placement strategy in timing-visible behavior:
consecutive K^T chunks and V tokens should map to adjacent columns so QK and
sparse PV can exploit row locality.

### Modeling Direction

- K^T placement:
  - Place consecutive `d_head` chunks in adjacent columns within a bank row.
  - Model QK MACs as bank-local accesses with high page locality.
- V placement:
  - Place consecutive sequence positions in adjacent columns.
  - Sparse PV accesses only the valid columns selected by the VOC-derived VDI.
- Batch placement:
  - Distribute batches across channels first.
  - Use remaining row space in the same bank after channel-level spreading.

### Implemented Config Parameters

- `enable_esdm`
- `enable_qk_act_pre`
- `kt_row_offset`
- `kt_col_start`
- `v_row_offset`
- `v_col_start`
- `esdm_columns_per_row`

### Implemented Statistics

- `sparepim_esdm_qk_rows`
- `sparepim_esdm_qk_page_hits`
- `sparepim_esdm_pv_rows`
- `sparepim_esdm_pv_page_hits`
- `sparepim_esdm_skipped_act_pre_cmds`

### Current Simplification

The current ESDM path models row/column locality and command timing. It does
not yet store actual K/V/wE tensor values in bank-resident storage.

### Status

Implemented for performance/page-hit modeling. Functional bank data placement is
planned in the next simulator layer.

## 5. Functional Dataflow Simulator And Golden Model

### Goal

Add value-level FP16 simulation so the SPARE-PIM path can be compared against a
C/C++ golden model.

### User Decisions

- No causal mask.
- Sequence lengths: `256`, `1024`, `2048`, `4096`.
- Q/K/V are FP16 random values across several ranges.
- Default random distribution: `uniform(-1, 1)` unless an experiment overrides
  it.
- `wE` is FP16 `uniform(0, 1)`.
- Square-root attention scaling is already fused into the weights.
- QK accumulation is cast back to FP16 after accumulation.
- PV accumulation uses FP32.
- Output tolerance: absolute and relative tolerance `1e-2`.
- Dumps are off by default; print only a summary unless explicitly enabled.

### Model Configurations

| Model | Layer | Head | KV head | Emb d | Seq len | Inter size | d_head |
|---|---:|---:|---:|---:|---:|---:|---:|
| GPT-baby | 6 | 6 | 6 | 384 | 256 | 1536 | 64 |
| GPT-3-small | 12 | 12 | 12 | 768 | 1024 | 3072 | 64 |
| TinyLlama | 22 | 32 | 4 | 2048 | 2048 | 5632 | 64 |
| Llama3.2-3B | 28 | 24 | 8 | 3072 | 4096 | 8192 | 128 |

For grouped-query attention, map an attention head to a KV head with:

```text
kv_head = floor(head * num_kv_heads / num_heads)
```

### Planned Components

- `BankStorage`
  - Holds bank-resident K, V, and wE data in ESDM layout.
  - Provides typed access by logical tensor index and physical bank/row/column.
- ESDM data loader
  - Places K^T along the `d_head` direction in adjacent bank columns.
  - Places V along the `seq_len` direction in adjacent bank columns.
  - Places Elemax weights so one-stream execution is possible.
- SPARE-PIM functional path
  - QK: FP16 multiply and accumulation, then FP16 cast.
  - ReLU: `relu_s = max(S, 0)`.
  - Bitmask: one bit per token, `1` when `relu_s > 0`.
  - VOC: count valid bits per bankgroup.
  - P: FP16 `wE * relu_s`.
  - Sparse PV: read only V entries selected by the bitmask/VDI.
  - Output: FP32 accumulation.
- Golden model
  - Dense Elemax-SDPA reference in C/C++.
  - Same random seed and FP16 conversion path as the SPARE-PIM model.
  - Compare output with `abs/rel <= 1e-2`.

### Status

Planned. Not implemented yet.

## 6. Validation Plan

### Build And Run

- Configure and build:

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

- Run the SPARE-PIM configuration:

```bash
./build/ramulator2 -f example_config_sparepim.yaml
```

Expected simulation behavior:

- Token-level ESEF command sequences are simulated.
- Nonzero VOC tokens update dynamic SPM latency.
- BGMU READ cycles follow `nCL + (N_BG - 1) * nCCDS + nBL`, where `N_BG`
  is the number of BGMU register groups read for the current pseudochannel-local
  sparse PV step.

### Corner Cases

- All-zero VOC:
  - `MOV` and `EMUL` still occur.
  - BGMU READ still occurs.
  - Sparse `ACT-SPM-PRE` is skipped.
  - `ACC` may still be issued according to configured accumulation behavior.
- Partial-zero VOC:
  - `SPM` latency uses `max(VOC)`, not `total_voc`.
- Invalid VOC metadata:
  - Negative values are clamped.
  - `max_voc > max_voc_per_bpu` is rejected.
  - `max_voc > total_voc` is rejected.
  - `total_voc > expected_voc_count * max_voc_per_bpu` is rejected.
- Large VOC:
  - ESDM row splitting is used when the active range exceeds the configured
    columns per row.

### Documentation Checks

- Keep `SPARE_paper_code_mapping.md` synchronized with code changes.
- Run `git diff --check` before committing.

## 7. Open Work (Functional Model Implementation)

To successfully implement the FP16 value-level simulation and golden model comparison, the remaining work is divided into the following detailed phases:

### Phase 1: Golden Model & Reference Data Generation
1. **Dense C/C++ Elemax-SDPA Golden Model**: Implement a standalone software reference model for ESEF (QxK^T, ReLU, bitmask, wE multiplication, sparse PV, FP32 accumulation).
2. **Random Tensor Generation**: Implement deterministic FP16 random generation (`uniform(-1, 1)` for activations, `uniform(0, 1)` for wE) to supply input data for Q, K, V, and wE.
3. **Reference Data Dump**: Add functionality to generate and save expected outputs, intermediate scores (S), and bitmasks for testing.

### Phase 2: Bank-Resident Storage & ESDM Loader
1. **BankStorage Component**: Create a `BankStorage` structure to hold FP16 data arrays within each physical bank.
2. **Physical Address Mapping**: Implement mapping from logical tensor indices (layer, head, token) to physical addresses (bank, row, column) using the ESDM policy.
3. **ESDM Data Loader**: Scatter the generated K^T, V, and wE reference tensors into the `BankStorage` before the simulation begins.

### Phase 3: Functional SPARE-PIM Dataflow Execution
1. **BPU & FSU Logic**: Implement the simulated FSU multipliers, adders, and Vector Buffers (VB-A, VB-B) to hold intermediate values.
2. **Command Execution Hooks**:
   - `MAC`: Fetch K^T from `BankStorage`, compute QK, accumulate in VB-B.
   - `MOV`/`EMUL`: Apply ReLU to VB-B, generate bitmask, multiply with wE, store P in VB-A.
   - `SPM`: Selectively read V from `BankStorage` using the bitmask (VDI), compute PV, and accumulate.
   - `ACC`: Transfer final partial-sums to the channel-level accumulator (CA) in FP32.
3. **DSSE Integration**: Ensure metadata (VDI, VOC) derived from the bitmask dictates the memory accesses during the `SPM` phase.

### Phase 4: Output Comparison & Debugging
1. **Automated Validation**: Compare the final outputs from the channel accumulators against the Golden Model outputs (checking `abs/rel <= 1e-2`).
2. **Debugging Utilities**: Add optional dump files to trace mismatched tensors, intermediate scores, or incorrect bitmasks.
3. **Integration Testing**: Run full validations on the four target model configurations (GPT-baby, GPT-3-small, TinyLlama, Llama3.2-3B).

## 8. Step-By-Step Progress

### Step 1: HBM3 Command And Timing Layer
Status: complete.

### Step 2: SPARE-PIM Controller
Status: complete.

### Step 3: SPAREPIMTrace Frontend
Status: complete.

### Step 4: ESDM Performance/Page-Hit Modeling
Status: complete.

### Step 5: Golden Model & Reference Data Generation (Phase 1)
Status: planned next.

### Step 6: Bank-Resident Storage & ESDM Loader (Phase 2)
Status: planned.

### Step 7: Functional SPARE-PIM Dataflow Execution (Phase 3)
Status: planned.

### Step 8: Output Comparison & Debugging (Phase 4)
Status: planned.
