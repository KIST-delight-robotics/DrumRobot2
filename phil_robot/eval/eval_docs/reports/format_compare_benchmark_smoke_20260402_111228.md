# format_compare_benchmark_smoke_20260402_111228.json 해설 리포트

- source_json: `reports/format_compare_benchmark_smoke_20260402_111228.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-02T11:12:28+09:00 |
| suite | 기록 없음 |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_smoke.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| 전체 결과 | 0/10 (0.0%) |

## 왜 이 실험을 했는가

`unknown` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정했습니다.
- planner 모델은 `qwen3:30b-a3b-instruct-2507-q4_K_M`로 고정했습니다.
- 케이스 입력은 `/home/shy/robot_project/phil_robot/eval/cases_smoke.json`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 10 |
| JSON 리포트 | reports/format_compare_benchmark_smoke_20260402_111228.json |
| Markdown 리포트 | eval_docs/reports/format_compare_benchmark_smoke_20260402_111228.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 10 |
| 통과 수 | 0 |
| 실패 수 | 10 |
| pass rate | 0/10 (0.0%) |
| 통과한 케이스 | 없음 |
| 실패한 케이스 | chat_greeting_basic, motion_wave_allowed, motion_wave_blocked_by_lock, motion_arm_up_basic, motion_left_arm_out_basic, play_tim_basic, status_question_basic, stop_request_basic, relative_wrist_raise_success, relative_wrist_raise_blocked |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |

### 바로 고쳐야 할 항목

| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |
| --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| motion_wave_allowed | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| motion_wave_blocked_by_lock | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| motion_arm_up_basic | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| motion_left_arm_out_basic | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| play_tim_basic | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| status_question_basic | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| stop_request_basic | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| relative_wrist_raise_success | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |
| relative_wrist_raise_blocked | 없음 | - | - | 기록 없음 | 실패 원인 기록 없음 |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 안녕 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| motion_wave_allowed | 손 흔들어줘 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| motion_wave_blocked_by_lock | 손 흔들어줘 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| motion_arm_up_basic | 팔 올려 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| motion_left_arm_out_basic | 왼팔 벌려 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| play_tim_basic | This Is Me 연주해줘 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| status_question_basic | 왜 멈췄어? | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| stop_request_basic | 종료해 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| relative_wrist_raise_success | 왼쪽 손목 더 올려 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |
| relative_wrist_raise_blocked | 거기서 50도 더 올리고 2초 있다 | 실패 | 없음 | - | - | 기록 없음 | 기록 없음 | 기록 없음 |

## 눈여겨볼 점

- 총 10건 중 0건 통과, 10건 실패였습니다. 실패 케이스는 `chat_greeting_basic, motion_wave_allowed, motion_wave_blocked_by_lock, motion_arm_up_basic, motion_left_arm_out_basic, play_tim_basic, status_question_basic, stop_request_basic, relative_wrist_raise_success, relative_wrist_raise_blocked`입니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `0/10 (0.0%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다.
