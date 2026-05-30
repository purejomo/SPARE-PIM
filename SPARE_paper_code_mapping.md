# SPARE-PIM Paper-To-Code Mapping

이 문서는 `Paper/SPARE.pdf`의 SPARE-PIM chapter가 현재 Ramulator 기반 코드에 어떻게 반영되어 있는지 정리한다. 현재 구현은 크게 두 층으로 나뉜다.

- 성능/timing simulator: 구현됨
- 실제 FP16 tensor dataflow functional simulator: 계획 수립 완료, 구현 예정

## 1. 구현 범위 요약

| 논문 항목 | 현재 코드 상태 | 주요 코드 |
|---|---|---|
| HBM3 hierarchy: channel, pCH, bankgroup, bank | 구현됨 | `src/dram/impl/HBM3.cpp` |
| SPARE-PIM RFU command: `MAC`, `EMUL`, `MOV`, `SPM`, `ACC` | 구현됨 | `src/dram/impl/HBM3.cpp` |
| Table I command timing: `MAC/EMUL=tCCDL`, `MOV=tCCDS`, `ACC=tCCDL+1`, dynamic `SPM` | 구현됨 | `src/dram/impl/HBM3.cpp`, `src/dram_controller/impl/sparepim_controller.cpp` |
| ESEF sequence: QK -> Elemax -> sparse PV -> ACC | 성능 모델 구현됨 | `src/dram_controller/impl/sparepim_controller.cpp` |
| VOC-based dynamic SPM latency | 구현됨 | `notify("sparepim_spm_latency", ...)` |
| BGMU READ latency equation | 구현됨, controller-local delay | `enqueue_bgmu_read_delay()` |
| `VOC=0` skip of `ACT-SPM-PRE` | 구현됨 | `expand_sparepim_token()` |
| ESDM K/V row-column mapping | 성능/page-hit 모델 구현됨 | `enqueue_qk_stage()`, `enqueue_sparse_pv_stage()` |
| VOC trace input and validation | 구현됨 | `SPAREPIMTrace` |
| FP16 Q/K/V/wE/S/P actual computation | 계획됨 | `SPARE_plan.md` |
| ReLU bitmask/VDI/VOC generated from actual `S` values | 계획됨 | `SPARE_plan.md` |
| Bank-resident K/V/wE data storage | 계획됨 | `SPARE_plan.md` |
| Golden vs SPARE-PIM functional output comparison | 계획됨 | `SPARE_plan.md` |

## 2. Hardware Architecture

### Paper

논문은 SPARE-PIM을 HBM 기반 PIM으로 정의한다. Channel은 pseudochannel, bankgroup, bank로 이어지며, bank에는 BPU, bankgroup에는 BGMU, logic die에는 pCH별 CA가 배치된다. BPU 내부에는 FSU, VB-A/VB-B, bitmask buffer, DSSE, local controller가 있다.

### Code Mapping

현재 Ramulator hierarchy는 `HBM3` DRAM model에 반영되어 있다.

- `src/dram/impl/HBM3.cpp`
  - levels: `channel`, `pseudochannel`, `bankgroup`, `bank`, `row`, `column`
  - organization override로 Table III류 구성을 YAML에서 지정

현재 BPU/BGMU/CA는 실제 hardware object로 독립 구현되어 있지는 않다. 대신 성능에 영향을 주는 동작은 controller와 DRAM command/timing layer에 모델링되어 있다.

- BPU all-bank command: `MAC`, `EMUL`, `MOV`, `SPM`
- single-bank CA transfer command: `ACC`
- BGMU READ: controller-local delay와 통계로 모델링
- CA accumulation: `ACC` command count/timing으로 모델링

### Status

부분 구현이다. hierarchy와 timing-visible behavior는 구현되어 있지만, FSU/VB/bitmask buffer/DSSE/CA의 실제 FP16 data storage와 value computation은 functional simulator 단계로 남아 있다.

## 3. SPARE-PIM Commands

### Paper

논문 Table I은 다섯 개 command를 정의한다.

- `MAC`: Q x K^T MAC
- `EMUL`: element-wise multiply
- `MOV`: VB와 bank 사이 data movement, optional ReLU
- `SPM`: VOC에 비례하는 sparse long-data command
- `ACC`: partial sum을 CA로 전달 및 누적

Timing은 다음과 같다.

- `MAC`: `tCCDL`
- `EMUL`: `tCCDL`
- `MOV`: `tCCDS`
- `SPM`: dynamic, proportional to VOC
- `ACC`: `tCCDL + 1`

### Code Mapping

`src/dram/impl/HBM3.cpp`에 command와 request translation이 추가되어 있다.

- command list에 `MAC`, `EMUL`, `MOV`, `SPM`, `ACC` 추가
- request list에 `mac`, `emul`, `mov`, `spm`, `acc`, `spare-pim` 추가
- command scope:
  - `MAC/EMUL/MOV/SPM`: bank-scope all-bank style address 사용
  - `ACC`: bank-scope single-bank command

Timing constraint도 HBM3 timing table에 추가되어 있다.

- `MAC`, `EMUL` -> `nCCDL`
- `MOV` -> `nCCDS`
- `ACC` -> `nCCDL + 1`
- `SPM` -> `nSPM`

`nSPM`은 고정 preset으로 두지 않고 controller가 매 SPM 전에 동적으로 설정한다.

```cpp
m_dram->notify("sparepim_spm_latency", op.cycles);
```

`HBM3::notify()`는 `nSPM` timing value와 SPM timing constraint entry를 갱신한다.

### Status

구현됨. 단, command가 실제 FP16 연산을 수행하지는 않고 timing/scheduling-visible event로 동작한다.

## 4. ESEF Execution Flow

### Paper

ESEF는 single-pass SDPA 흐름이다.

1. QK^T 계산
   - `MAC` 반복 발행
   - 결과 `S`는 VB-B에 누적
2. Elemax 활성화
   - `MOV` with ReLU
   - bitmask 생성
   - DSSE가 VDI/VOC 생성
   - `EMUL`로 `P = wE * ReLU(S)`
3. Sparse PV
   - controller가 BGMU READ로 VOC를 읽음
   - `max(VOC)`에 따라 SPM latency 설정
   - `VOC=0`이면 sparse sequence skip
4. ACC
   - partial sum을 CA에서 누적

### Code Mapping

`src/dram_controller/impl/sparepim_controller.cpp`의 `expand_sparepim_token()`이 `spare-pim` request를 내부 command queue로 확장한다.

흐름은 다음과 같다.

1. `enqueue_qk_stage(base_addr, mac_commands)`
2. `MOV`
3. `EMUL`
4. `enqueue_bgmu_read_delay()`
5. `max_voc == 0`이면 skip
6. `enqueue_sparse_pv_stage(base_addr, max_voc)`
7. `ACC` 반복
8. request callback complete

`SPAREPIMTrace` frontend는 trace에서 `max_voc`, `total_voc`, `mac_commands`, `acc_commands`를 만들어 `Request::scratchpad`에 넣는다.

```cpp
req.scratchpad[0] = t.max_voc;
req.scratchpad[1] = t.total_voc;
req.scratchpad[2] = t.mac_commands;
req.scratchpad[3] = t.acc_commands;
```

### Status

성능 모델 기준 구현됨. 실제 `S`, `P`, bitmask, VDI 값 생성은 functional model 단계에서 구현 예정이다.

## 5. Dynamic Sparsity, ReLU, Bitmask, DSSE

### Paper

논문에서는 `MOV` 시점에 ReLU가 수행되고, VB-B sign bit 기반 bitmask가 생성된다. DSSE는 bitmask에서 VDI를 추출하고 VOC를 계산한다. 이 metadata generation은 Elemax의 `EMUL` 시간과 overlap되어 추가 latency가 없다고 설명한다.

### Code Mapping

현재 성능 simulator는 actual `S` value를 계산하지 않으므로 ReLU/bitmask/VDI를 값 기반으로 만들지 않는다. 대신 frontend trace가 VOC를 제공하고, controller가 이 VOC를 SPARE-PIM metadata 결과로 사용한다.

구현된 부분:

- `MOV` command가 ESEF 흐름에 포함됨
- `EMUL` command가 ESEF 흐름에 포함됨
- metadata generation latency는 별도 delay 없이 `EMUL`과 overlap된 것으로 모델링
- VOC는 `SPAREPIMTrace`에서 입력 및 검증
- `max_voc`가 SPM dynamic latency와 skip 여부를 결정

검증/validation:

- VOC list entry 수가 bankgroup 수와 다르면 error
- `max_voc > seq_len`이면 error
- `max_voc > total_voc`이면 error
- 음수 VOC는 0으로 clamp

### Status

부분 구현이다. 논문의 timing effect는 구현되어 있지만, 실제 ReLU가 `S` 값에 적용되어 bitmask를 생성하는 functional behavior는 아직 구현되지 않았다. `SPARE_plan.md`의 functional simulator 단계에서 `ReLU(S)`, bitmask, VDI, VOC를 golden과 비교하도록 계획되어 있다.

## 6. BGMU READ And SPM Control

### Paper

논문 Algorithm 1은 BGMU READ와 sparse kernel issue flow를 정의한다.

1. 각 BGMU에서 VOC를 읽음
2. `max_count = max(VOC)` 계산
3. `max_count == 0`이면 `ACT-SPM-PRE` skip
4. 아니면 all-bank ACT, SPM, PRE 발행

BGMU READ latency는 다음과 같다.

```text
L_BGMU = tCL + (N_BG - 1) * tCCD_S + tBURST
```

### Code Mapping

`SPAREPIMController::enqueue_bgmu_read_delay()`에서 위 식을 cycle 단위로 구현한다.

```cpp
int bgmu_read_latency = nCL + (m_num_bgs - 1) * nCCDS + nBL;
```

이 delay는 controller-local `Delay` op로 들어가며, memory/controller tick과 같은 DRAM cycle 단위로 진행된다. 즉 1GHz config에서 1 cycle은 1ns이다.

Table III override 예제에서는:

```text
nCL=18, nCCDS=1, nBL=4, N_BG=8
L_BGMU = 18 + 7*1 + 4 = 29 cycles
```

`VOC=0` skip은 `expand_sparepim_token()`에서 처리한다.

- `sparepim_skipped_spm`
- `sparepim_esdm_skipped_act_pre_cmds`

같은 통계로 확인 가능하다.

### Status

구현됨. 단, BGMU READ는 실제 `RD` command를 bankgroup별로 발행하지 않고 controller-local delay로 모델링한다. 논문 latency equation과 command dependency는 반영되어 있지만, DRAM row state/RD bus contention을 BGMU READ command로 세밀하게 업데이트하지는 않는다.

## 7. ESDM Data Mapping

### Paper

ESDM은 K^T와 V를 DRAM bank에 배치해 ESEF가 page-hit 중심으로 수행되게 한다.

- K^T:
  - `d_head` dimension chunks가 같은 bank row의 adjacent columns에 배치
  - QK 계산이 bank-local로 끝남
- V:
  - `L_seq` direction chunks가 adjacent columns에 배치
  - valid P에 대응하는 V만 읽음
- wE:
  - S/P token index와 alignment되도록 같은 bank에 배치
- batch:
  - channel 우선 분산
  - 부족하면 row space에 순차 배치

### Code Mapping

현재 ESDM은 `SPAREPIMController`에서 address generation과 page-hit 통계로 구현되어 있다.

Config:

- `enable_esdm`
- `enable_qk_act_pre`
- `kt_row_offset`, `kt_col_start`
- `v_row_offset`, `v_col_start`
- `esdm_columns_per_row`

QK stage:

- `enqueue_qk_stage()`
- `MAC` command가 K^T row의 adjacent column을 따라 발행됨
- row column 수를 넘으면 다음 row로 split
- `sparepim_esdm_qk_rows`
- `sparepim_esdm_qk_page_hits`

Sparse PV stage:

- `enqueue_sparse_pv_stage()`
- `max_voc`가 V row capacity를 넘으면 row 단위로 SPM split
- `sparepim_esdm_pv_rows`
- `sparepim_esdm_pv_page_hits`

### Status

성능/page-hit 모델 기준 구현됨. 실제 bank storage에 K/V/wE FP16 data를 배치하고 VDI가 해당 data를 읽는 functional behavior는 아직 구현 예정이다.

## 8. Table II And Table III Parameters

### Paper

Table II는 model configuration을 제시한다.

| Model | Layer | Head | KV head | Emb d | Seq len | Inter size | Derived d_head |
|---|---:|---:|---:|---:|---:|---:|---:|
| GPT-baby | 6 | 6 | 6 | 384 | 256 | 1536 | 64 |
| GPT-3-small | 12 | 12 | 12 | 768 | 1024 | 3072 | 64 |
| TinyLlama | 22 | 32 | 4 | 2048 | 2048 | 5632 | 64 |
| Llama3.2-3B | 28 | 24 | 8 | 3072 | 4096 | 8192 | 128 |

Table III는 HBM3 timing/organization을 제시한다.

### Code Mapping

Table II는 `SPARE_plan.md`에 functional/golden model preset으로 정리되어 있다. TinyLlama와 Llama3.2-3B는 `num_kv_heads < num_heads`이므로 GQA mapping을 사용하도록 계획했다.

Table III는 새 preset을 만들지 않고 YAML override로 반영한다.

- `example_config_sparepim.yaml`
  - `nCL: 18`
  - `nRCDRD/nRCDWR: 14`
  - `nRP: 14`
  - `nRAS: 34`
  - `nCCDS: 1`
  - `nCCDL: 2`
  - `nRRDL: 6`
  - `nFAW: 30`

### Status

Table III timing은 성능 simulator에 반영됨. Table II는 functional simulator preset 계획에 반영됨.

## 9. Frontend And Example Workload

### Code Mapping

`SPAREPIMTrace`는 VOC trace를 읽어 `spare-pim` request를 생성한다.

지원 trace format:

```text
VOC voc0,voc1,... [addr] [mac_cmds] [acc_cmds]
TOKEN max_voc total_voc [addr] [mac_cmds] [acc_cmds]
max_voc total_voc [addr] [mac_cmds] [acc_cmds]
```

예제:

- `example_config_sparepim.yaml`
- `example_sparepim.trace`

실행:

```bash
./build/ramulator2 -f example_config_sparepim.yaml
```

검증된 대표 통계:

- `sparepim_tokens_0: 3`
- `sparepim_bgmu_read_cycles_0: 87`
- `sparepim_spm_cmds_0: 3`
- `sparepim_spm_cycles_0: 9`
- `sparepim_skipped_spm_0: 1`

### Status

구현됨.

## 10. Not Yet Implemented: Functional Dataflow

현재 코드가 아직 하지 않는 일은 명확하다.

- FP16 random Q/K/V/wE generation
- K/V/wE를 실제 bank storage에 저장
- QK MAC으로 실제 `S` 생성
- `S`에 ReLU 적용
- ReLU 결과에서 bitmask 생성
- bitmask에서 VDI/VOC 생성
- sparse PV가 VDI에 해당하는 V만 읽어 FP32 output accumulate
- golden dense Elemax-SDPA와 SPARE-PIM functional output 비교

이는 `SPARE_plan.md`의 `Functional Dataflow Simulator And Golden Model` 섹션에 다음 단계로 정리되어 있다.

확정된 functional policy:

- causal mask 없음
- `Q/K/V`: FP16 random, 여러 range/distribution mode 지원
- 기본 random mode: `uniform(-1, 1)`
- `wE`: FP16 `uniform(0, 1)`
- QK: accumulate 후 FP16 cast
- PV: FP32 accumulation
- tolerance: `abs_tol=1e-2`, `rel_tol=1e-2`
- dump: 기본 off, stdout summary만 출력

## 11. Overall Assessment

현재 구현은 논문의 성능 시뮬레이션 핵심 경로를 반영한다.

구현된 핵심:

- RFU command set
- command timing
- in-order ESEF sequence
- BGMU READ latency
- VOC-based dynamic SPM latency
- `VOC=0` skip
- ESDM row/column access and page-hit stats
- Table III timing override
- VOC trace frontend and validation

아직 남은 핵심:

- 실제 FP16 dataflow
- ReLU/bitmask/VDI/VOC value generation
- bank-resident K/V/wE storage
- golden model comparison

따라서 현재 코드는 SPARE-PIM의 performance/timing simulator로는 동작하며, 논문 dataflow의 functional correctness 검증은 다음 구현 단계로 남아 있다.
