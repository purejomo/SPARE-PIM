# SPARE-PIM Simulator Implementation Plan

이 문서는 Ramulator 2.1 기반 SPARE-PIM simulator 구현 계획을 정리한다. 목표는 Elemax 기반 SDPA 실행 흐름(ESEF), 동적 sparsity 활용, BGMU/VOC 기반 SPM latency 조절, 그리고 ESDM data mapping 효과를 성능 시뮬레이션에 반영하는 것이다.

## 0. 구현 원칙

- 기존 Ramulator 2.x 구조를 최대한 유지한다.
- 일반 `HBM3` 동작과 SPARE-PIM 실험 경로가 섞여도 빌드가 깨지지 않도록 변경 범위를 작게 유지한다.
- 논문 Table III timing 값은 새 preset을 강제하지 않고 YAML override로 반영한다.
- SPARE-PIM 전용 동작은 controller/frontend/config에서 명시적으로 켜는 방식으로 구성한다.
- 성능 시뮬레이터가 1차 목표이므로 실제 FP16 data value 계산보다 command timing, scheduling, metadata latency, sparsity skip, page-hit 모델링을 우선한다.

## 1. DRAM/HBM3 Command And Timing Layer

### 구현 내용

- `src/dram/impl/HBM3.cpp`에 SPARE-PIM RFU command 추가
  - `MAC`: all-bank MAC command
  - `EMUL`: all-bank element-wise multiply command
  - `MOV`: VB/bank movement and optional ReLU command
  - `SPM`: all-bank sparse long-data command
  - `ACC`: single-bank channel accumulator transfer command
- SPARE-PIM request translation 추가
  - `mac -> MAC`
  - `emul -> EMUL`
  - `mov -> MOV`
  - `spm -> SPM`
  - `acc -> ACC`
- timing field `nSPM` 추가
  - 기본값은 1 cycle
  - 실제 SPM latency는 controller가 `max(VOC)`를 계산한 뒤 매 SPM 전에 동적으로 갱신
- `IDRAM::notify()` hook 사용
  - key: `sparepim_spm_latency`
  - value: controller가 계산한 dynamic SPM latency
- Table I 반영
  - `MAC`, `EMUL`: `tCCDL`
  - `MOV`: `tCCDS`
  - `ACC`: `tCCDL + 1`
  - `SPM`: `nSPM`, controller가 동적 설정

### 완료 상태

- `HBM3.cpp`에 command/timing hook 추가 완료
- build 통과
- Table III override smoke test 통과

## 2. SPARE-PIM Controller

### 목표

SPARE-PIM의 memory controller 상호작용을 명시적으로 모델링한다. 특히 BGMU READ, VOC aggregation, `VOC=0` skip, dynamic SPM latency 설정이 controller 내부에서 처리되어야 한다.

### 구현 파일

- 새 파일: `src/dram_controller/impl/sparepim_controller.cpp`
- 수정 파일: `src/dram_controller/CMakeLists.txt`

### Controller 동작

1. workload/frontend가 token 단위 SPARE-PIM work item을 controller에 전달한다.
2. controller는 ESEF sequence를 내부 command queue로 확장한다.
3. QK stage
   - `MAC` 반복 발행
   - 반복 횟수는 `d_head / fsu_width` 또는 config의 `num_mac_commands`로 결정
4. Elemax stage
   - `MOV` 발행
   - MOV 시점에 ReLU bitmask 생성이 발생한 것으로 모델링
   - DSSE의 VDI/VOC 생성은 EMUL latency와 overlap된다고 가정
   - `EMUL` 발행
5. BGMU READ stage
   - controller가 channel 내 bankgroup 수만큼 BGMU READ를 수행한 것으로 모델링
   - latency:
     - `L_BGMU = nCL + (N_BG - 1) * nCCDS + nBL`
   - BGMU READ 자체는 기존 RD command를 실제로 모두 발행하는 방법과 controller-local delay로 모델링하는 방법이 있다.
   - 1차 구현은 controller-local delay counter로 처리하고, 이후 필요하면 RD command sequence로 세분화한다.
6. Sparse PV stage
   - controller가 BGMU VOC list에서 `max_voc` 계산
   - `max_voc == 0`이면 `ACT-SPM-PRE` 전체 skip
   - `max_voc > 0`이면:
     - target row ACT
     - `m_dram->notify("sparepim_spm_latency", spm_latency)` 호출
     - `SPM` 발행
     - PRE 발행
   - SPM latency 모델:
     - 기본: `spm_latency = max_voc * spm_cycles_per_voc`
     - `spm_cycles_per_voc`는 config parameter
7. Accumulation stage
   - partial sum이 여러 bank 또는 pseudochannel에서 발생하는 경우 `ACC` 발행
   - `ACC` latency는 HBM3 timing layer의 `tCCDL + 1`

### 1차 request ABI

Frontend가 `Request`를 생성할 때 type은 `spare-pim` request id를 사용한다. controller는 `Request::scratchpad`를 아래처럼 해석한다.

- `scratchpad[0]`: `max_voc`
- `scratchpad[1]`: `total_voc`
- `scratchpad[2]`: token별 `MAC` command count override. 0이면 config 기본값 사용
- `scratchpad[3]`: token별 `ACC` command count override. 0이면 config 기본값 사용

이 ABI는 3단계 `SPAREPIMTrace` frontend에서 사용한다. VOC list 전체 저장은 후속 ESDM/page-hit 정밀화 단계에서 확장한다.

### 필요한 config parameter

- `spm_cycles_per_voc`: VOC 1개당 SPM cycle
- `mac_commands_per_token`: QK stage MAC command 반복 횟수
- `acc_commands_per_token`: ACC command 반복 횟수
- `bgmu_read_as_delay`: BGMU READ를 local delay로 처리할지 여부
- `enable_act_spm_pre`: SPM 전후 ACT/PRE 발행 여부
- `queue_size`: controller 입력 queue 크기

### 통계

- `sparepim_tokens`
- `sparepim_mac_cmds`
- `sparepim_mov_cmds`
- `sparepim_emul_cmds`
- `sparepim_spm_cmds`
- `sparepim_acc_cmds`
- `sparepim_bgmu_reads`
- `sparepim_bgmu_read_cycles`
- `sparepim_spm_cycles`
- `sparepim_skipped_spm`
- `sparepim_total_voc`
- `sparepim_max_voc_sum`

### 현재 구현 상태

- `SPAREPIM` controller 등록 완료
- `spare-pim` request를 ESEF command queue로 확장
- `BGMU READ`는 controller-local delay로 모델링
- `VOC=0`이면 sparse `ACT-SPM-ACC-PRE` sequence skip
- `max_voc>0`이면 `sparepim_spm_latency` notify로 SPM latency 동적 설정
- `MAC/MOV/EMUL/SPM/ACC/ACT/PRE`, BGMU, VOC 관련 통계 등록

## 3. Frontend And Workload Input

### 목표

추론 성능 평가를 위해 token/head/layer 단위 workload를 입력받아 SPARE-PIM controller에 전달한다.

### 구현 후보

- 새 frontend: `SPAREPIMTrace`
- 입력 trace line 예시:
  - `TOKEN layer head batch q_idx voc0,voc1,...,vocN`
  - 또는 단순형: `TOKEN max_voc voc0,voc1,...`

### 1차 구현 trace 형식

`SPAREPIMTrace`는 아래 형식을 지원한다.

- `VOC voc0,voc1,... [addr] [mac_cmds] [acc_cmds]`
  - VOC list에서 `max_voc`와 `total_voc`를 자동 계산
- `TOKEN max_voc total_voc [addr] [mac_cmds] [acc_cmds]`
  - 이미 집계된 VOC를 직접 입력
- `TOKEN layer head batch q_idx voc0,voc1,... [mac_cmds] [acc_cmds]`
  - layer/head/batch/q_idx 필드는 현재 주소 생성에는 사용하지 않고, VOC list만 집계
- `max_voc total_voc [addr] [mac_cmds] [acc_cmds]`
  - bare compact 형식
- `path` 없이 `num_tokens`, `fixed_max_voc`, `fixed_total_voc`를 주면 fixed synthetic trace 생성

Frontend는 `spare-pim` request를 발행하고 callback 완료까지 기다린다. 이 때문에 simulation이 request enqueue 시점에서 조기 종료되지 않고 controller의 ESEF command queue 완료까지 진행된다.

### 지원해야 할 workload mode

- 실제 VOC trace 입력
- sparsity ratio 기반 synthetic VOC 생성
- fixed VOC 입력
- all-zero VOC corner case

### 모델 config

- `num_layers`
- `num_heads`
- `num_kv_heads`
- `seq_len`
- `d_head`
- `batch_size`
- `fsu_width = 16`
- `pbank`

### 현재 구현 상태

- `SPAREPIMTrace` frontend 등록 완료
- VOC list trace와 집계형 trace 지원
- `d_head / fsu_width`로 기본 `MAC` command count 계산
- callback 기반 completion wait 구현
- `VOC=0` skip과 `max_voc>0` dynamic SPM path end-to-end smoke test 완료

## 4. ESDM Address Mapping And Page-Hit Model

### 목표

K/V 배치에 따른 page-hit 및 ACT/PRE 감소 효과를 반영한다.

### 모델링 방향

- K^T
  - `d_head` 방향 consecutive chunk가 같은 bank row의 adjacent columns에 배치
  - QK MAC은 bank-local page-hit 중심으로 모델링
- V
  - `L_seq` 방향 consecutive chunk가 adjacent columns에 배치
  - valid P index에 대응하는 V column만 접근
- batch 증가 시
  - channel 우선 분산
  - channel 공간 부족 시 bank row 공간에 순차 배치

### 구현 방식

- 1차: SPARE-PIM controller가 command address vector를 직접 생성
- 2차: 별도 `SPAREPIMMapper` 또는 helper class로 분리
- page-hit 통계:
  - SPM ACT/PRE 발행 횟수
  - skipped ACT/PRE 횟수
  - row hit/miss/conflict와 별도 SPARE-PIM 전용 page stats

### 1차 구현 config

`SPAREPIM` controller에 아래 config를 추가한다.

- `enable_esdm`: ESDM row/column address generation 사용 여부
- `enable_qk_act_pre`: QK `MAC` stage에도 K^T row open/close를 모델링할지 여부
- `kt_row_offset`, `kt_col_start`: K^T row/column 시작 위치
- `v_row_offset`, `v_col_start`: V row/column 시작 위치
- `esdm_columns_per_row`: ESDM에서 한 row에 연속 배치되는 chunk 수. 기본값은 DRAM column 수

### 현재 구현 상태

- QK stage `MAC` command가 K^T row의 adjacent column을 따라 발행됨
- `MAC` command 수가 row column 수를 넘으면 다음 row로 넘어가며 ACT/PRE를 한 번 더 발행
- sparse PV stage는 V row의 adjacent column을 따라 `SPM`을 발행
- `max_voc`가 row column 수를 넘으면 row 단위로 `SPM`을 split
- split된 `SPM`의 총 sparse work latency는 `sum(chunk_voc * spm_cycles_per_voc)`로 유지
- `VOC=0`이면 skipped ACT/PRE command 수를 별도 통계로 집계
- 추가 통계:
  - `sparepim_esdm_qk_rows`
  - `sparepim_esdm_qk_page_hits`
  - `sparepim_esdm_pv_rows`
  - `sparepim_esdm_pv_page_hits`
  - `sparepim_esdm_skipped_act_pre_cmds`

## 5. Functional Dataflow Simulator And Golden Model

### 목표

현재 simulator는 command timing, VOC 기반 skip, ESDM page-hit를 모델링하지만 실제 tensor 값을 계산하지 않는다. 다음 단계에서는 SPARE-PIM 내부 dataflow를 C/C++ functional model로 확장하여, random FP16 input에 대해 golden Elemax-SDPA 결과와 SPARE-PIM dataflow 결과를 비교한다.

### Causal Mask Policy

현재 검증 목표는 정해진 sequence에 대해 SDPA 1회 추론을 수행하는 것이므로 causal mask는 사용하지 않는다. Functional/golden model은 full attention으로 고정한다.

### Tensor Shape And Supported Lengths

우선 논문 evaluation 길이에 맞춰 아래 `seq_len`만 지원/검증한다.

- `256`
- `1024`
- `2048`
- `4096`

Table II의 LLM configuration을 functional/golden model preset으로 제공한다.

| Model | Layer | Head | KV head | Emb d | Seq len | Inter size | Derived d_head |
|---|---:|---:|---:|---:|---:|---:|---:|
| GPT-baby | 6 | 6 | 6 | 384 | 256 | 1536 | 64 |
| GPT-3-small | 12 | 12 | 12 | 768 | 1024 | 3072 | 64 |
| TinyLlama | 22 | 32 | 4 | 2048 | 2048 | 5632 | 64 |
| Llama3.2-3B | 28 | 24 | 8 | 3072 | 4096 | 8192 | 128 |

기본 tensor shape는 config 또는 preset으로 지정한다.

- `model`: `GPT-baby`, `GPT-3-small`, `TinyLlama`, `Llama3.2-3B`
- `batch`
- `num_layers`
- `num_heads`
- `num_kv_heads`
- `seq_len`
- `d_head = emb_d / num_heads`
- `fsu_width = 16`
- `pbank`

`num_kv_heads < num_heads`인 TinyLlama/Llama3.2-3B는 GQA로 처리한다. Query head `h`는 `kv_head = floor(h * num_kv_heads / num_heads)`의 K/V head를 참조한다.

### Precision Rule

사용자 지정 규칙을 따른다.

- `Q`, `K`, `V`, `S`, `P`, `wE`: FP16
- QK:
  - FP16 multiply
  - accumulation 후 FP16 cast하여 `S` 저장
  - `sqrt(d_head)` scaling은 이미 weight에 fusion된 것으로 가정하므로 simulator에서 적용하지 않음
- Elemax:
  - `ReLU(S)` 수행
  - bitmask는 ReLU 결과가 0이 아닌 token을 1로 저장
  - `P = FP16(wE * ReLU(S))`
- PV:
  - FP16 multiply
  - FP32 accumulation
  - output은 FP32

### Data Generation

초기 functional test는 deterministic random data를 생성한다.

- seed는 config에서 지정
- `Q`, `K`, `V`: FP16 random
  - 여러 range/distribution mode를 지원한다.
  - 기본은 numerical sanity를 위해 `uniform(-1, 1)`로 둔다.
  - 추가 mode 예: `uniform(-0.1, 0.1)`, `uniform(-4, 4)`, `normal(0, 1)`
- `wE`: FP16 `uniform(0, 1)`
- generated input과 output dump 여부를 config로 제어

### Golden Model

golden model은 SPARE-PIM mapping과 무관하게 dense logical tensor layout에서 계산한다.

1. `S[i][j] = cast_fp16(sum_d Q[i][d] * K[j][d])`
2. `R[i][j] = ReLU(S[i][j])`
3. `golden_bitmask[i][j] = R[i][j] != 0`
4. `P[i][j] = cast_fp16(wE[i][j] * R[i][j])`
5. `O[i][d] = sum_j P[i][j] * V[j][d]` with FP32 accumulation

### SPARE-PIM Functional Model

SPARE-PIM functional model은 실제 bank-resident dataflow를 모사한다.

1. ESDM mapper가 `K`, `V`, `wE`를 bank storage에 배치한다.
2. QK stage:
   - VB-A에 `Q` chunk load
   - bank의 `K^T` chunk를 ESDM address로 읽음
   - BPU/FSU가 MAC 수행
   - 결과 `S`를 VB-B에 FP16으로 저장
3. Elemax stage:
   - `MOV` with ReLU flag 시점에 VB-B의 sign/value를 검사
   - negative score는 0으로 clamp
   - token당 1 bit bitmask 생성
   - DSSE가 bitmask에서 VDI list와 VOC를 생성
   - BGMU에 bankgroup별 VOC 저장
   - `EMUL`로 `wE * ReLU(S)`를 계산하여 `P`를 VB-A에 저장
4. Sparse PV stage:
   - controller가 BGMU VOC를 읽어 `max_voc` 결정
   - `max_voc=0`이면 sparse PV skip
   - `max_voc>0`이면 DSSE VDI 순서대로 valid `P`와 corresponding `V`만 읽음
   - FSU가 FP16 multiply, FP32 accumulate 수행
5. ACC stage:
   - bank/pCH partial sum을 CA에서 FP32 accumulate
   - 최종 output FP32 생성

### Bitmask And VOC Rule

- bitmask granularity는 token당 1 bit
- `ReLU(S[j]) > 0`이면 bit `1`
- `ReLU(S[j]) == 0`이면 bit `0`
- negative score는 ReLU 이후 0이므로 bit `0`
- `VOC = popcount(bitmask)` per BPU/BGMU aggregation scope
- BGMU는 bankgroup 내 BPU VOC를 집계하고 controller가 BGMU READ로 조회

### Bank Storage Model

functional simulator에는 실제 data storage model을 추가한다.

- `BankStorage` abstraction
  - hierarchy: channel, pseudochannel, bankgroup, bank, row, column
  - row/column address로 FP16 vector chunk read/write
- `K^T` mapping
  - `d_head` 방향 chunk를 같은 bank row의 adjacent column에 연속 배치
  - 한 row capacity 초과 시 다음 row로 이동
- `V` mapping
  - `L_seq` 방향 token chunk를 adjacent column에 연속 배치
  - VDI가 가리키는 valid token만 column access
- `wE` mapping
  - `S/P` token index와 alignment되도록 같은 bank/row policy로 배치
- batch/head distribution
  - channel 우선 분산
  - channel 부족 시 bank row 공간에 순차 배치

### Comparison Program

별도 executable 또는 simulator mode를 추가한다.

- 후보 파일:
  - `src/sparepim/functional/*`
  - `src/sparepim/tools/sparepim_functional_test.cpp`
- 출력:
  - golden output FP32
  - SPARE-PIM functional output FP32
  - max absolute error
  - max relative error
  - bitmask mismatch count
  - VOC mismatch count
- pass/fail threshold:
  - FP16/FP32 mixed precision을 고려하여 config로 tolerance 지정
  - 기본 `abs_tol = 1e-2`, `rel_tol = 1e-2`
  - dump는 기본 off이며 stdout summary만 출력

### Required User Inputs

사용자가 제공한 Table II에서 model별 `num_layers`, `num_heads`, `num_kv_heads`, `d_head`, `seq_len`은 확정되었다. 추가 정책도 아래처럼 확정한다.

- causal mask 없음
- `Q/K/V`: FP16 random, 다양한 range/distribution mode 지원
- `wE`: FP16 `uniform(0, 1)`
- comparison tolerance: `abs_tol = 1e-2`, `rel_tol = 1e-2`
- dump: 기본 off, stdout summary만 출력

### Implementation Steps

1. FP16 helper 추가
   - C++ `_Float16` 또는 portable half library 사용 여부 결정
   - cast/rounding 규칙 통일
2. tensor generator 추가
3. dense golden Elemax-SDPA 구현
4. bank storage 및 ESDM mapper 구현
5. SPARE-PIM functional dataflow 구현
6. ReLU, bitmask, VDI, VOC 생성 검증
7. golden vs SPARE-PIM output 비교 도구 추가
8. `seq_len` 256/1024/2048/4096 smoke/regression test 추가

## 6. Validation Plan

### Build

- `cmake -S . -B build`
- `cmake --build build -j$(nproc)`
- `git diff --check`

### Smoke Tests

- HBM3 normal read path가 깨지지 않는지 확인
- SPARE-PIM controller가 빈/작은 trace를 처리하는지 확인
- `VOC=0`일 때 `SPM` command count가 증가하지 않는지 확인
- `max(VOC)>0`일 때 `SPM` latency가 `max_voc * spm_cycles_per_voc`로 반영되는지 확인
- BGMU READ latency가 `nCL + (N_BG - 1) * nCCDS + nBL`로 누적되는지 확인

### Example Run

예제 파일:

- `example_config_sparepim.yaml`
- `example_sparepim.trace`

실행:

```bash
./build/ramulator2 -f example_config_sparepim.yaml
```

현재 예제는 3 token을 포함한다.

- token 0: all-zero VOC, sparse `ACT-SPM-PRE` skip
- token 1: `max_voc=3`, one-row SPM
- token 2: `max_voc=6`, `esdm_columns_per_row=4`에서 two-row SPM split

검증된 주요 통계:

- `sparepim_tokens_0: 3`
- `sparepim_bgmu_reads_0: 24`
- `sparepim_bgmu_read_cycles_0: 87`
- `sparepim_spm_cmds_0: 3`
- `sparepim_spm_cycles_0: 9`
- `sparepim_skipped_spm_0: 1`
- `sparepim_esdm_qk_rows_0: 3`
- `sparepim_esdm_qk_page_hits_0: 9`
- `sparepim_esdm_pv_rows_0: 3`
- `sparepim_esdm_pv_page_hits_0: 6`

### Corner Cases

- 모든 bankgroup VOC가 0
- 한 bankgroup만 큰 VOC
- VOC list 길이가 bankgroup 수와 다름
- `seq_len`이 `fsu_width`로 나누어떨어지지 않음
- multiple channel/batch 분산

### 현재 corner case 처리

- `VOC=0`: sparse `ACT-SPM-PRE` skip, skip 통계 기록
- `max_voc`가 ESDM row column 수 초과: V row 단위로 `SPM` split
- `d_head`가 `fsu_width`로 나누어떨어지지 않음: `ceil(d_head / fsu_width)`로 `MAC` command count 계산
- 음수 VOC: parser에서 0으로 clamp
- VOC list 길이 불일치: `validate_voc_count=true`일 때 DRAM `bankgroup` 수와 비교해 configuration error 발생
- `seq_len` 초과 VOC: `seq_len > 0`일 때 `max_voc > seq_len`이면 configuration error 발생
- `max_voc > total_voc`: 집계형 trace에서 `total_voc > 0`이면 configuration error 발생
- `total_voc=0`이고 `max_voc>0`: controller 통계에서 `total_voc=max_voc`로 보정

## 7. Current Open Decisions

- BGMU READ를 실제 RD command sequence로 발행할지, controller-local delay로 둘지
  - 현재 계획: 1차는 local delay, 이후 필요하면 세분화
- SPM 내부 MAC pipeline latency 공식
  - 현재 계획: `max_voc * spm_cycles_per_voc`
  - 필요 시 fixed setup/drain latency 추가
- 실제 bitmask/VDI value를 저장할지 여부
  - functional model에서는 bitmask/VDI/VOC를 모두 저장하고 golden과 비교
- multi-die/8-stack 전체 모델링 범위
  - 현재 Ramulator channel hierarchy 기준으로 먼저 구현하고, stack/die 차원은 config와 통계 naming에서 확장
- FP16 구현 방식
  - `_Float16`, compiler intrinsic, 또는 portable half header 중 하나를 선택해야 함

## 8. 단계별 진행 순서

1. DRAM command/timing hook 추가
2. SPARE-PIM controller 골격 구현
3. synthetic/manual request로 controller unit smoke test
4. SPARE-PIM trace frontend 구현
5. ESDM address generation 추가
6. 예제 config/trace 추가
7. 논문 Table III 기반 validation config 정리
8. 성능 통계 출력 정리
9. functional golden model 추가
10. bank storage/ESDM data placement 추가
11. SPARE-PIM functional dataflow와 output comparison 추가
