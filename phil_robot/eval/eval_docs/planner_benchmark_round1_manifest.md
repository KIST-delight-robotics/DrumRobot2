# planner_benchmark_round1_manifest.json 해설 리포트

- source_json: `planner_benchmark_round1_manifest.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | planner 비교 계획표 |
| round_name | planner_model_round1 |
| suite | smoke |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| candidate 수 | 7 |
| auto_pull | 예 |
| 사람 검토 전 자동 실행만 하고 멈춤 | 예 |
| JSON 전용 경로 사용 | 예 |

## 왜 이 실험을 했는가

planner 후보를 같은 조건에서 비교하기 전에, 어떤 모델을 어떤 방식으로 불러오고 어떤 기준으로 맞춰 놓을지 보여주는 실행 계획표입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 결과를 고정하고 planner만 비교하도록 설계했습니다.
- planner 입력 JSON을 고정해 모델별 비교 조건을 맞췄습니다.
- 문자열 형식 비교가 아니라 JSON 전용 경로만 보도록 설정했습니다.
- 자동 실행 뒤에는 바로 사람 검토 단계로 넘기도록 멈추게 했습니다.

## 테스트 구성

| 순번 | 후보 분류 | 요청 태그 | 실제로 pull할 후보 | pull tag 수 |
| --- | --- | --- | --- | --- |
| 1 | qwen3_moe | qwen3:30b-a3b-q4_K_M | qwen3:30b-a3b-instruct-2507-q4_K_M, qwen3:30b-a3b-q4_K_M | 2 |
| 2 | qwen3_dense | qwen3:32b-q4_K_M | qwen3:32b-q4_K_M | 1 |
| 3 | nemotron_moe | nemotron-cascade-2:30b-a3b-q4_K_M | nemotron-cascade-2:30b-a3b-q4_K_M | 1 |
| 4 | gpt_oss | gpt-oss:20b | gpt-oss:20b | 1 |
| 5 | mistral_small | mistral-small3.2:24b-instruct-250… | mistral-small3.2:24b-instruct-2506-q4_K_M | 1 |
| 6 | exaone | exaone3.5:32b-instruct-q4_K_M | exaone3.5:32b-instruct-q4_K_M | 1 |
| 7 | lfm2 | lfm2:24b-q4_K_M | lfm2:24b-q4_K_M | 1 |

## 눈여겨볼 점

- 후보마다 pull tag 목록이 달라, 같은 family 안에서도 실제로 가져오려는 태그가 둘 이상일 수 있습니다.
- classifier 결과와 planner 입력을 고정하는 설정이 들어 있어, 모델 자체 차이에 집중하려는 계획표입니다.

## 종합 총평

이 문서는 결과표는 아니지만, 비교 조건을 어떻게 맞춰 놓았는지 한눈에 보여 줍니다. 나중에 결과 문서를 읽을 때 “무엇을 바꿔 본 비교였는가”를 되짚는 기준으로 쓰기 좋습니다.
