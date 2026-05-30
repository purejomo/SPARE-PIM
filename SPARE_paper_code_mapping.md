# SPARE-PIM Paper-To-Code Mapping

This document explains how the SPARE-PIM chapter in `Paper/SPARE.pdf` maps to
the current Ramulator-based implementation.

The implementation currently has two layers:

- Performance/timing simulator: implemented
- FP16 tensor-value functional simulator: planned

## 1. Implementation Summary

| Paper item | Current status | Main code |
|---|---|---|
| HBM3 hierarchy: channel, pseudochannel, bankgroup, bank | Implemented through the HBM3 model | `src/dram/impl/HBM3.cpp` |
| SPARE-PIM RFU commands: `MAC`, `EMUL`, `MOV`, `SPM`, `ACC` | Implemented | `src/dram/impl/HBM3.cpp` |
| Command timing: `MAC/EMUL=tCCDL`, `MOV=tCCDS`, `ACC=tCCDL+1`, dynamic `SPM` | Implemented | `src/dram/impl/HBM3.cpp`, `src/dram_controller/impl/sparepim_controller.cpp` |
| ESEF flow: QK -> Elemax -> sparse PV -> ACC | Implemented as a performance model | `src/dram_controller/impl/sparepim_controller.cpp` |
| VOC-based dynamic SPM latency | Implemented | `notify("sparepim_spm_latency", ...)` |
| BGMU READ latency equation | Implemented as controller-local delay | `enqueue_bgmu_read_delay()` |
| `VOC=0` skip of sparse `ACT-SPM-PRE` | Implemented | `expand_sparepim_token()` |
| ESDM K/V row-column mapping | Implemented as timing/page-hit model | `enqueue_qk_stage()`, `enqueue_sparse_pv_stage()` |
| VOC trace input and validation | Implemented | `src/frontend/impl/sparepim/sparepim_trace.cpp` |
| Actual FP16 Q/K/V/wE/S/P computation | Planned | `SPARE_plan.md` |
| ReLU bitmask and VOC generated from actual S values | Planned | `SPARE_plan.md` |
| Bank-resident K/V/wE data storage | Planned | `SPARE_plan.md` |
| Golden output comparison | Planned | `SPARE_plan.md` |

## 2. Hardware Architecture

### Paper Model

SPARE-PIM is organized as an HBM-based PIM system. The hierarchy is:

```text
HBM stack
  DRAM die
    Channel
      Pseudochannel
        Bankgroup with BGMU
          Bank with BPU
Logic die with channel-level accumulators
```

Each BPU contains an FSU, VB-A, VB-B, a bitmask buffer, DSSE, and a local
controller. Each BGMU aggregates VOC metadata from the BPUs in its bankgroup.
The logic die contains channel-level accumulators.

### Code Mapping

The Ramulator HBM3 model already provides the timing hierarchy:

- `channel`
- `pseudochannel`
- `bankgroup`
- `bank`
- `row`
- `column`

The current code does not instantiate BPU, BGMU, DSSE, bitmask buffer, or CA as
separate hardware objects. Instead, their timing-visible effects are modeled in
the controller and DRAM command layer:

- BPU all-bank work: `MAC`, `EMUL`, `MOV`, `SPM`
- BGMU READ: controller-local delay and statistics
- DSSE metadata generation: overlapped with `EMUL`
- CA accumulation: `ACC` commands and `tCCDL + 1` timing

### Status

Partially implemented. Hierarchy and timing-visible behavior are modeled. Actual
FP16 data storage and value computation remain planned work.

## 3. SPARE-PIM Commands

### Paper Model

The paper defines five PIM commands:

- `MAC`: Q x K^T multiply-accumulate
- `EMUL`: element-wise multiply
- `MOV`: data movement between bank and vector buffers, with optional ReLU
- `SPM`: sparse long-data command whose latency depends on VOC
- `ACC`: partial-sum transfer to the channel-level accumulator

Timing:

- `MAC`: `tCCDL`
- `EMUL`: `tCCDL`
- `MOV`: `tCCDS`
- `SPM`: dynamic, proportional to VOC
- `ACC`: `tCCDL + 1`

### Code Mapping

`src/dram/impl/HBM3.cpp` adds the commands and request names:

- Commands: `MAC`, `EMUL`, `MOV`, `SPM`, `ACC`
- Requests: `mac`, `emul`, `mov`, `spm`, `acc`, `spare-pim`

The HBM3 timing table includes the corresponding constraints:

- `MAC` and `EMUL` use `nCCDL`
- `MOV` uses `nCCDS`
- `ACC` uses `nCCDL + 1`
- `SPM` uses `nSPM`

`nSPM` is not a fixed preset value. The controller updates it before each sparse
kernel:

```cpp
m_dram->notify("sparepim_spm_latency", op.cycles);
```

The HBM3 notify handler updates both the `nSPM` value and the active SPM timing
constraint.

### Status

Implemented for timing/scheduling. The commands do not yet perform FP16 value
computation.

## 4. ESEF Execution Flow

### Paper Model

ESEF executes SDPA in one pass:

1. QK^T
   - Repeated `MAC` commands compute the score vector S.
2. Elemax
   - `MOV` applies ReLU and creates the bitmask.
   - DSSE extracts VDI and counts VOC.
   - `EMUL` computes `P = wE * ReLU(S)`.
3. Sparse PV
   - The controller reads BGMU VOC metadata.
   - `max(VOC)` controls SPM latency.
   - If `max(VOC) == 0`, sparse `ACT-SPM-PRE` is skipped.
4. ACC
   - Partial sums are accumulated in the logic die.

### Code Mapping

`SPAREPIMController::expand_sparepim_token()` expands a token-level
`spare-pim` request into the command queue:

1. `enqueue_qk_stage(base_addr, mac_commands)`
2. `MOV`
3. `EMUL`
4. `enqueue_bgmu_read_delay()`
5. Skip sparse PV when `max_voc == 0`
6. `enqueue_sparse_pv_stage(base_addr, max_voc)` when `max_voc > 0`
7. Repeated `ACC`
8. Complete the request callback

The frontend places metadata in the request scratchpad:

```cpp
req.scratchpad[0] = t.max_voc;
req.scratchpad[1] = t.total_voc;
req.scratchpad[2] = t.mac_commands;
req.scratchpad[3] = t.acc_commands;
```

### Status

Implemented as a performance model. Actual S, P, bitmask, and VDI values are not
generated yet.

## 5. Elemax, ReLU, Bitmask, And DSSE

### Paper Model

Elemax is:

```text
Elemax(s_i) = w_E(i) * ReLU(s_i)
```

ReLU immediately converts negative scores to zero. The sign information creates
a bitmask, and DSSE uses that bitmask to produce:

- VDI: valid data index list
- VOC: valid output count

This metadata generation overlaps with the Elemax `EMUL` command.

### Code Mapping

The current timing simulator does not compute actual score values. Therefore,
ReLU and bitmask generation are represented by timing events and trace-provided
VOC metadata:

- `MOV` appears at the correct point in ESEF.
- `EMUL` appears after `MOV`.
- DSSE metadata generation adds no separate latency.
- VOC is provided by `SPAREPIMTrace`.
- `max_voc` drives dynamic SPM latency and zero-VOC skip.

Validation currently happens on the metadata input:

- VOC list length can be checked against the BPU count per pseudochannel.
- Negative VOC entries are clamped to zero.
- `max_voc > max_voc_per_bpu` is rejected.
- `max_voc > total_voc` is rejected.

### Status

Partially implemented. Timing effects are modeled. Value-level ReLU, bitmask,
VDI, and VOC generation are planned for the functional simulator.

## 6. BGMU READ And SPM Control

### Paper Model

The sparse kernel control path is:

1. Read VOC from each BGMU.
2. Compute `max_count = max(VOC)`.
3. If `max_count == 0`, skip `ACT-SPM-PRE`.
4. Otherwise issue all-bank ACT, SPM, and PRE.

BGMU READ latency:

```text
L_BGMU = tCL + (N_BG - 1) * tCCD_S + tBURST
```

### Code Mapping

`SPAREPIMController::enqueue_bgmu_read_delay()` implements the equation:

```cpp
int bgmu_read_latency = nCL + (m_num_bgs - 1) * nCCDS + nBL;
```

This is modeled as a controller-local delay in DRAM cycles. It is synchronized
with the controller/memory clock. For a 1 GHz configuration, one cycle is 1 ns.

The delay intentionally does not issue real `RD` commands, so it does not update
DRAM row state or consume regular read-command scheduling slots. It models the
paper's BGMU metadata-read cost while keeping BGMU as controller-visible
metadata rather than normal bank data.

The zero-VOC skip path updates:

- `sparepim_skipped_spm`
- `sparepim_esdm_skipped_act_pre_cmds`

### Status

Implemented.

## 7. Dynamic SPM Latency

### Paper Model

`SPM` is a sparse long-data command. Its latency is proportional to the maximum
VOC among the bankgroups participating in the sparse PV operation.

### Code Mapping

The controller computes:

```text
spm_latency = spm_fixed_cycles + max_voc * spm_cycles_per_voc
```

Then it calls:

```cpp
m_dram->notify("sparepim_spm_latency", spm_latency);
```

The HBM3 model updates `nSPM` before the `SPM` command is scheduled.

### Status

Implemented.

## 8. ESDM Data Mapping

### Paper Model

ESDM places data to maximize internal locality:

- K^T: consecutive `d_head` elements are placed in adjacent columns within the
  same bank.
- V: consecutive sequence positions are placed in adjacent columns.
- Batch growth first uses different channels, then remaining row space.

This reduces ACT/PRE overhead and increases page hits.

### Code Mapping

The current controller has an ESDM-oriented address generator:

- `enqueue_qk_stage()` models K^T column progression for QK MACs.
- `enqueue_sparse_pv_stage()` models V column progression for sparse PV.
- Row splitting occurs when the configured column span is exceeded.

Relevant config parameters:

- `enable_esdm`
- `enable_qk_act_pre`
- `kt_row_offset`
- `kt_col_start`
- `v_row_offset`
- `v_col_start`
- `esdm_columns_per_row`

Relevant statistics:

- `sparepim_esdm_qk_rows`
- `sparepim_esdm_qk_page_hits`
- `sparepim_esdm_pv_rows`
- `sparepim_esdm_pv_page_hits`
- `sparepim_esdm_skipped_act_pre_cmds`

### Status

Implemented for performance/page-hit modeling. Actual bank-resident K/V/wE data
placement is planned for the functional model.

## 9. Trace Frontend

### Paper Need

To evaluate inference speed, the simulator needs token-level sparsity metadata
that represents the result of Elemax and DSSE.

### Code Mapping

`SPAREPIMTrace` supports VOC-list and aggregate-token inputs:

```text
VOC voc0,voc1,... [addr] [mac_cmds] [acc_cmds]
TOKEN max_voc total_voc [addr] [mac_cmds] [acc_cmds]
TOKEN layer head batch q_idx voc0,voc1,... [mac_cmds] [acc_cmds]
max_voc total_voc [addr] [mac_cmds] [acc_cmds]
```

It also supports fixed synthetic generation through config values:

- `num_tokens`
- `fixed_max_voc`
- `fixed_total_voc`

The frontend waits for controller callbacks, so simulation completion reflects
the full ESEF command sequence rather than only request enqueue.

### Status

Implemented.

## 10. Simulation Configuration

`example_config_sparepim.yaml` provides a compact SPARE-PIM simulation setup:

- Frontend: `SPAREPIMTrace`
- Controller: `SPAREPIM`
- DRAM: `HBM3`
- Trace: `example_sparepim.trace`
- ESDM enabled
- BGMU READ modeled as controller-local delay
- SPM latency set by `spm_cycles_per_voc`

The trace contains per-BPU VOC metadata:

- one token-level work item per line
- one VOC count per BPU participating in a pseudochannel-local sparse PV step
- four BGMU registers per pseudochannel, each holding four bank/BPU VOC values
- each VOC count is bounded by the FSU width, which is 16 by default

This drives dynamic SPM latency and BGMU READ accounting without embedding a
full per-token bitmask in the trace.

## 11. Functional Model Gap

The following paper behaviors are not implemented yet:

- Store random FP16 K, V, and wE in bank-resident ESDM layout.
- Compute QK score values.
- Apply ReLU to S.
- Generate one-bit-per-token bitmask.
- Generate VDI and VOC from the bitmask.
- Compute `P = wE * ReLU(S)`.
- Run sparse PV using only valid V entries.
- Accumulate the output in FP32.
- Compare against a dense C/C++ golden Elemax-SDPA model.

These items are planned in `SPARE_plan.md`.

## 12. Current Assessment

The current code implements the paper's timing-visible SPARE-PIM mechanisms:

- RFU command path
- ESEF command ordering
- BGMU READ latency
- VOC-based dynamic SPM latency
- zero-VOC sparse-kernel skip
- ESDM page-locality model
- trace-driven token input

The remaining major step is to add value-level FP16 functional simulation and
golden-model validation.
