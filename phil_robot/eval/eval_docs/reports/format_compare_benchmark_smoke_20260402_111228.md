# format_compare_benchmark_smoke_20260402_111228.json 해설 리포트

- source_json: `reports/format_compare_benchmark_smoke_20260402_111228.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | planner 출력 형식 비교 |
| generated_at | 2026-04-02T11:12:28+09:00 |
| case_count | 10 |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| temperature | 0 |

## 왜 이 실험을 했는가

planner 출력 형식을 문자열 방식과 JSON 방식으로 나눠서, 속도와 파싱 편의가 어떻게 달라지는지 보려는 비교입니다.

## 이번에 바꿔 보거나 고정한 점

- 모델은 그대로 두고 출력 형식만 `legacy_str`와 `json`으로 바꿨습니다.
- classifier는 케이스마다 한 번만 돌리고 같은 intent 결과를 두 형식 비교에 재사용했습니다.
- planner 두 형식 모두 warm-up 한 번 뒤 시간을 쟀습니다.
- temperature는 `0`로 고정했습니다.

## 결과 요약

| 지표 | 문자열 형식 | JSON 형식 |
| --- | --- | --- |
| parse 성공률 | 100.0% | 100.0% |
| 평균 wall 시간 | 3.945 s | 4.880 s |
| 평균 prompt token | 1461.4 | 1529.4 |
| 평균 eval token | 37.3 | 57.1 |
| 평균 출력 글자 수 | 64.7 | 126.2 |
| 평균 명령 수 | 2.5 | 1.1 |

## 상세 표

| case id | 상황 | 문자열 형식 시간 | JSON 형식 시간 | 차이 | 말 내용 동일 | 명령 수 변화 |
| --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 인사 요청 | 2.642 s | 4.011 s | 1.369 s | 같음 | 0 -> 0 |
| motion_wave_allowed | 움직일 수 있는 상태에서 손 흔들기 | 3.352 s | 3.943 s | 0.591 s | 다름 | 1 -> 1 |
| motion_wave_blocked_by_lock | 안전 키가 잠긴 상태에서 손 흔들기 | 4.097 s | 4.024 s | -0.073 s | 같음 | 2 -> 1 |
| motion_arm_up_basic | 팔 올리기 | 3.062 s | 4.317 s | 1.255 s | 다름 | 1 -> 1 |
| motion_left_arm_out_basic | 왼팔 벌리기 | 4.273 s | 4.971 s | 0.699 s | 다름 | 4 -> 1 |
| play_tim_basic | 곡 연주 요청 | 5.927 s | 5.191 s | -0.735 s | 다름 | 8 -> 2 |
| status_question_basic | 오류로 멈춘 뒤 이유 묻기 | 3.654 s | 5.692 s | 2.037 s | 다름 | 0 -> 0 |
| stop_request_basic | 종료 요청 | 5.191 s | 5.012 s | -0.179 s | 다름 | 6 -> 2 |
| relative_wrist_raise_success | 왼쪽 손목을 조금 더 올리기 | 4.418 s | 6.016 s | 1.598 s | 다름 | 1 -> 1 |
| relative_wrist_raise_blocked | 손목을 더 올리면 범위를 넘는 상황 | 2.838 s | 5.625 s | 2.787 s | 다름 | 2 -> 2 |

## 눈여겨볼 점

- `손목을 더 올리면 범위를 넘는 상황` 장면에서는 JSON 형식이 문자열 형식보다 `2.787 s` 더 걸렸습니다.
- `오류로 멈춘 뒤 이유 묻기` 장면에서는 JSON 형식이 문자열 형식보다 `2.037 s` 더 걸렸습니다.
- `왼쪽 손목을 조금 더 올리기` 장면에서는 JSON 형식이 문자열 형식보다 `1.598 s` 더 걸렸습니다.
- 이 문서는 속도만 보는 것이 아니라, 출력 구조가 더 길어져도 사람이 읽고 후처리하기 쉬워지는지를 함께 보는 비교입니다.

## 종합 총평

이번 비교에서는 두 형식 모두 파싱에는 성공했지만, JSON 형식이 더 긴 응답과 더 긴 실행 시간을 보였습니다. 대신 응답 구조가 명확해져 이후 처리에는 유리하므로, 속도와 가독성 사이의 교환 관계를 보여 주는 문서로 읽는 편이 맞습니다.
