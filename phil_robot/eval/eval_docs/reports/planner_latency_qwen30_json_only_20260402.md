# planner_latency_qwen30_json_only_20260402.json 해설 리포트

- source_json: `reports/planner_latency_qwen30_json_only_20260402.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | planner 시간 분리 측정 |
| generated_at | 2026-04-02T16:32:55+09:00 |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| warm-up 횟수 | 1 |
| 실측 횟수 | 2 |
| 평균 시간 | 1.584 s |
| p95 시간 | 1.815 s |

## 왜 이 실험을 했는가

planner 자체 속도를 따로 보기 위해 classifier와 입력을 고정하고 planner만 여러 번 불러 본 실험입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 결과를 케이스마다 한 번만 만들고 그대로 다시 사용했습니다.
- planner 입력 JSON도 고정해 입력 차이 없이 planner만 비교했습니다.
- 케이스마다 warm-up 1회 뒤, 실측 2회를 기록했습니다.
- 첫 호출의 차가운 시작 시간도 따로 기록했습니다.

## 결과 요약

| case id | 상황 | input chars | avg | median | p95 | 서로 다른 raw 수 | 서로 다른 말 수 | 서로 다른 명령 수 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 인사 요청 | 858 | 1.412 s | 1.412 s | 1.456 s | 2 | 1 | 1 |
| motion_wave_allowed | 움직일 수 있는 상태에서 손 흔들기 | 877 | 1.757 s | 1.757 s | 1.815 s | 2 | 1 | 1 |

## 눈여겨볼 점

- 첫 호출 시간은 `인사 요청`에서 `1.579 s`로 기록됐습니다.
- 평균 시간이 가장 긴 장면은 `움직일 수 있는 상태에서 손 흔들기`였습니다.
- raw 응답 문자열은 조금 달라도, 실제로 남는 명령 수와 말 내용 종류가 크게 흔들리지 않는지도 같이 보게 했습니다.

## 종합 총평

이 문서는 planner 자체 시간을 따로 떼어 본 결과라서, 전체 파이프라인보다 순수 planner 속도와 출력 흔들림을 읽기에 좋습니다. 속도뿐 아니라 같은 입력에서 답이 얼마나 일정한지도 같이 볼 수 있다는 점이 핵심입니다.
