# Phil Robot Classifier 벤치마크 리포트

## 1. 목적

이 문서는 Phil robot LLM control pipeline의 classifier 모델 선정 실험을 정리한 것입니다.

목표 운영 제약:

- classifier 지연시간 목표: `<= 1.5 s`
- planner 모델: `qwen3:30b-a3b-instruct-2507-q4_K_M`로 고정
- 평가 프로토콜: [cases_smoke.json](/home/shy/robot_project/phil_robot/eval/cases_smoke.json)의 `smoke` suite

선정 기준:

- 현재 control/task 분포에서 classifier 정확도 유지
- 1단계 intent classification latency 감소
- 현재 multi-layer 경로
  `STT -> classifier -> domain planner -> validator -> executor -> TTS`
  에서 end-to-end 안정성 유지

## 2. 실험 설정

### 2.1 평가 데이터셋

- suite: `smoke`
- case 수: `11`
- case 범주:
  - greeting / chat
  - motion request
  - play request
  - stop request
  - status question
  - relative-motion edge case
  - safety lock / busy-state blocking
  - general knowledge chat
  - robot identity question

### 2.2 지표

- `classifier_checks`
  - suite에 정의된 classifier 기대값의 exact-match pass 수
- `cases`
  - classifier, planner, validator, speech check를 모두 포함한 full end-to-end pass 수
- `avg_classifier_sec`
  - 1단계 classifier 평균 지연시간
- `avg_planner_sec`
  - 2단계 planner 평균 지연시간
- `avg_total_sec`
  - turn 당 LLM 총 지연시간 평균

### 2.3 고정 파이프라인 설정

- planner 모델: `qwen3:30b-a3b-instruct-2507-q4_K_M`
- validator / executor: 현재 저장소 구현 사용
- state 입력: [state_adapter.py](/home/shy/robot_project/phil_robot/pipeline/state_adapter.py)의 reduced LLM state summary 사용

## 3. 1차 광범위 classifier 스크리닝

첫 번째 실험은 `smoke` suite에 대해 classifier-only evaluation을 수행하며 다양한 instruct 계열 로컬 모델을 선별했습니다.

| Model | Size | Classifier Checks | Avg (s) | Max (s) | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| `llama3.1:8b-instruct-q5_K_M` | 5.7 GB | 11/11 | 4.678 | 5.972 | Baseline, accurate but slow |
| `llama3.2:3b-instruct-q4_K_M` | 2.0 GB | 10/11 | 4.270 | 5.981 | general-knowledge chat 오분류 |
| `phi3:3.8b-mini-4k-instruct-q4_K_S` | 2.2 GB | 5/11 | 8.889 | 23.182 | Korean intent classification 불안정 |
| `gemma:2b-instruct` | 1.6 GB | 9/11 | 5.680 | 12.469 | greeting / knowledge chat 정확도 손실 |
| `qwen2.5:3b-instruct` | 1.9 GB | 10/11 | 3.742 | 9.774 | 강한 초기 후보 |
| `qwen2.5:7b-instruct` | 4.7 GB | 11/11 | 5.567 | 14.588 | 정확하지만 stage 1 용도로는 느림 |

## 4. Quantized Qwen 중심 스크리닝

두 번째 실험은 목표 operating point에 더 가까운 explicit Qwen quantized 후보를 집중 비교했습니다.

| Model | Size | Classifier Checks | Avg (s) | Median (s) | Max (s) | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `qwen2.5:0.5b-instruct-q4_K_M` | 397 MB | 4/11 | 2.020 | 1.696 | 5.052 | 정확도 부족 |
| `qwen2.5:1.5b-instruct-q4_K_M` | 986 MB | 8/11 | 2.239 | 1.991 | 4.920 | 더 빠르지만 여전히 불안정 |
| `qwen2.5:3b-instruct-q4_K_M` | 1.9 GB | 10/11 | 2.672 | 2.368 | 5.659 | 괜찮은 fallback 후보 |
| `qwen3:4b-instruct-2507-q4_K_M` | 2.5 GB | 11/11 | 1.863 | 1.583 | 4.685 | 종합 최적 classifier 후보 |

## 5. Multi-Layer End-to-End Benchmark

아래 실험은 classifier만이 아니라 전체 control stack을 평가한 결과입니다. Planner는 계속 `qwen3:30b-a3b-instruct-2507-q4_K_M`으로 고정했습니다.

| Classifier Model | Cases | Classifier Checks | Planner Checks | Validator Checks | E2E Checks | Avg Classifier (s) | Avg Planner (s) | Avg Total (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `llama3.1:8b-instruct-q5_K_M` | 11/11 | 11/11 | 15/15 | 11/11 | 10/10 | 6.481 | 6.121 | 12.602 |
| `llama3.2:3b-instruct-q4_K_M` | 9/11 | 9/11 | 13/15 | 11/11 | 10/10 | 4.491 | 7.800 | 12.291 |
| `qwen2.5:3b-instruct` | 10/11 | 10/11 | 14/15 | 11/11 | 10/10 | 2.733 | 4.802 | 7.535 |
| `qwen3:4b-instruct-2507-q4_K_M` | 11/11 | 11/11 | 15/15 | 11/11 | 10/10 | 1.627 | 4.717 | 6.344 |

비고:

- 최종 `qwen3:4b` 시간은 모델 선정 후 standalone run에서 다시 측정해 parallel-process 간섭을 줄였습니다.
- `qwen3:4b`는 다음 조건을 동시에 만족한 유일한 classifier였습니다.
  - `11/11` classifier correctness
  - `11/11` full-case pass rate
  - 평균 classifier latency `2 s` 미만

## 6. 최종 선정

선정된 classifier:

- `qwen3:4b-instruct-2507-q4_K_M`

선정 이유:

1. 낮은 지연 후보 중 가장 높은 classifier 정확도
2. 정확한 후보들 중 가장 좋은 end-to-end total latency
3. 현재 `smoke` suite에서 회귀 없음
4. 이전 baseline `llama3.1:8b-instruct-q5_K_M` 대비 stage 1 latency가 크게 감소

이전 baseline 대비 latency 감소:

- classifier latency: `6.481 s -> 1.627 s`
- 절대 감소량: `4.854 s`
- 상대 감소율: `74.9%`

이전 baseline 대비 end-to-end LLM latency 감소:

- total LLM latency: `12.602 s -> 6.344 s`
- 절대 감소량: `6.258 s`
- 상대 감소율: `49.7%`

## 7. 목표와의 차이 분석

원래 stage-1 classification 목표는 `<= 1.5 s`였습니다.

현재 선정 모델:

- `avg_classifier_sec = 1.627 s`
- `median_classifier_sec = 1.617 s`

해석:

- 선정 모델은 목표에 매우 가깝지만, mean latency 기준으로는 아직 완전히 충족하지는 못합니다.
- 남은 차이는 모델 크기보다 아키텍처 최적화로 줄이는 편이 더 적절합니다.

권장 다음 단계:

- classifier 앞에 lightweight rule-based prefilter 추가
  - greeting
  - identity/name question
  - simple stop phrases
  - obvious general-knowledge chat

기대 효과:

- 불필요한 classifier 호출 감소
- 평균 classifier latency를 `1.5 s` 아래로 낮출 가능성
- 현재 validator / planner stack을 유지하면서 정확도 보존

## 8. 산출물

- evaluation suite: [cases_smoke.json](/home/shy/robot_project/phil_robot/eval/cases_smoke.json)
- 선택된 classifier 기준 최신 smoke report:
  [smoke_report_qwen3_4b_classifier.json](/home/shy/robot_project/phil_robot/eval/reports/smoke_report_qwen3_4b_classifier.json)
- 현재 classifier 설정:
  [intent_classifier.py](/home/shy/robot_project/phil_robot/pipeline/intent_classifier.py)
