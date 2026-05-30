# SPARE-PIM Simulator

This repository is a research fork of Ramulator 2.1 / Ramulator2 used to model
SPARE-PIM on top of the existing HBM3 simulation framework. The upstream
Ramulator codebase provides the DRAM hierarchy, timing infrastructure, request
flow, and YAML-based configuration system. This fork focuses on the SPARE-PIM
execution path.

## Current Scope

- HBM3 RFU-style PIM commands: `MAC`, `EMUL`, `MOV`, `SPM`, and `ACC`
- A `SPAREPIM` memory controller that expands one token-level request into the
  ESEF flow: QK, Elemax, BGMU READ, sparse PV, and ACC
- VOC-driven dynamic `SPM` latency
- `VOC=0` skip of the sparse `ACT-SPM-PRE` sequence
- ESDM-oriented K/V column placement and page-hit statistics
- A `SPAREPIMTrace` frontend for synthetic or trace-driven inference timing
  experiments

The simulator integrates both a **performance/timing model** and an **exact FP16 functional dataflow simulator**:
- Contains a standalone C++ Golden Model for exact IEEE-754 FP16 Elemax-SDPA computations.
- Simulates physical bank-resident memory (`BankStorage`) with realistic ESDM (Elemax-based SDPA Data Mapping) row-column layouts.
- Executes token-by-token tensor operations alongside the simulated trace, mimicking true hardware MAC, MOV, EMUL, and sparse SPM behavior.
- Validates the hardware functional outputs against the Golden Model at the end of the simulation.

## Build

```bash
cd /home/ghlee/SPARE-PIM
cmake -S . -B build
cmake --build build -j$(nproc)
```

The build produces the standalone simulator at:

```bash
./build/ramulator2
```

## Run A SPARE-PIM Simulation

```bash
cd /home/ghlee/SPARE-PIM
./build/ramulator2 -f example_config_sparepim.yaml
```

The configuration uses:

- `SPAREPIMTrace` frontend
- `SPAREPIM` controller
- HBM3 timing overrides in YAML
- `example_sparepim.trace` as a compact per-BPU VOC metadata trace

Useful output statistics include:

- `sparepim_trace_tokens`
- `sparepim_mac_cmds`
- `sparepim_mov_cmds`
- `sparepim_emul_cmds`
- `sparepim_bgmu_reads`
- `sparepim_bgmu_read_cycles`
- `sparepim_spm_cmds`
- `sparepim_spm_cycles`
- `sparepim_skipped_spm`
- `sparepim_acc_cmds`

## Functional Simulator Configuration

To enable the FP16 functional dataflow verification, make sure `enable_functional: true` is set under the `Controller` configuration in your YAML file.

```yaml
  Controller:
    impl: SPAREPIM
    enable_functional: true
    # Model parameters for generating reference data
    seq_len: 256
    d_head: 64
    kt_row_offset: 0
    v_row_offset: 100
```
*Note: Ensure that `kt_row_offset` and `v_row_offset` are spaced far enough apart to prevent physical row overlaps during simulated `BankStorage` mapping!*

When enabled, you will see a validation summary at the end of the simulation run:
```text
==========================================
[SPARE-PIM] Finalizing Functional Simulation
[SPARE-PIM] Status: PASSED (Outputs match Golden Model for 4 tokens)
==========================================
```

## Trace Format

`SPAREPIMTrace` accepts several compact token formats.

```text
VOC voc0,voc1,... [addr] [mac_cmds] [acc_cmds]
TOKEN max_voc total_voc [addr] [mac_cmds] [acc_cmds]
TOKEN layer head batch q_idx voc0,voc1,... [mac_cmds] [acc_cmds]
max_voc total_voc [addr] [mac_cmds] [acc_cmds]
```

For example:

```text
VOC 9,12,7,10,8,11,13,6,14,9,10,12,7,8,11,15 0x0
VOC 4,6,5,7,3,8,6,5,9,4,7,6,5,3,8,4 0x40
```

The `VOC` list provides BPU-level sparsity metadata. The controller uses this to compute `max_count`, dynamically setting the `SPM(max_count)` latency to simulate zero-skipping.

## Important Files

- `example_config_sparepim.yaml`: Minimal SPARE-PIM experiment configuration
- `example_sparepim.trace`: Per-BPU VOC metadata trace
- `src/dram/impl/HBM3.cpp`: HBM3 command and timing extensions
- `src/dram_controller/impl/sparepim_controller.cpp`: SPARE-PIM controller
- `src/frontend/impl/sparepim/sparepim_trace.cpp`: SPARE-PIM trace frontend
- `SPARE_plan.md`: Implementation plan and remaining work
- `SPARE_paper_code_mapping.md`: Mapping between the paper model and the code

## Notes On Upstream Ramulator

This fork keeps the Ramulator structure and build system so that standard DRAM
models and existing Ramulator concepts remain available. For general Ramulator
usage, architecture details, or citation information, refer to the upstream
Ramulator2 project.
