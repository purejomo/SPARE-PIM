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

The current simulator is primarily a performance/timing model. The FP16
functional dataflow model and golden-output comparison flow are planned in
`SPARE_plan.md`.

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

## Run The SPARE-PIM Example

```bash
cd /home/ghlee/SPARE-PIM
./build/ramulator2 -f example_config_sparepim.yaml
```

The example uses:

- `SPAREPIMTrace` frontend
- `SPAREPIM` controller
- HBM3 timing overrides in YAML
- `example_sparepim.trace` as a small VOC trace

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
VOC 0,0,0,0,0,0,0,0 0x0
VOC 1,3,2,0,1,0,0,1 0x40
VOC 6,1,0,0,0,0,0,0 0x80
```

The frontend computes `max_voc` and `total_voc`, stores them in the request
scratchpad, and waits for the controller callback so the simulation ends only
after the full ESEF command queue completes.

## Important Files

- `example_config_sparepim.yaml`: Minimal SPARE-PIM experiment configuration
- `example_sparepim.trace`: Small VOC trace for smoke testing
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
