# smoke_report_q3-4b-q4km_q3-32b-q4km_20260402_1527.json 해설 리포트

- source_json: `reports/smoke_report_q3-4b-q4km_q3-32b-q4km_20260402_1527.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-02T15:27:45+09:00 |
| suite | smoke |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_smoke.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:32b-q4_K_M |
| 전체 결과 | 10/13 (76.9%) |

## 왜 이 실험을 했는가

`smoke` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정했습니다.
- planner 모델은 `qwen3:32b-q4_K_M`로 고정했습니다.
- 케이스 입력은 `/home/shy/robot_project/phil_robot/eval/cases_smoke.json`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 13 |
| JSON 리포트 | reports/smoke_report_q3-4b-q4km_q3-32b-q4km_20260402_1527.json |
| Markdown 리포트 | eval_docs/reports/smoke_report_q3-4b-q4km_q3-32b-q4km_20260402_1527.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 13 |
| 통과 수 | 10 |
| 실패 수 | 3 |
| pass rate | 10/13 (76.9%) |
| 통과한 케이스 | chat_greeting_basic, motion_wave_allowed, motion_wave_blocked_by_lock, motion_arm_up_basic, motion_left_arm_out_basic, play_tim_basic, status_question_basic, relative_wrist_raise_success, chat_general_knowledge, chat_identity_name |
| 실패한 케이스 | stop_request_basic, relative_wrist_raise_blocked, motion_blocked_while_playing |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 13/13 (100.0%) |
| 계획 선택 | 17/17 (100.0%) |
| 명령 검사 | 11/13 (84.6%) |
| 최종 발화 | 8/10 (80.0%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |

### 바로 고쳐야 할 항목

| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |
| --- | --- | --- | --- | --- | --- |
| stop_request_basic | 발화 표현 누락 | 발화 표현 누락: 종료, 안녕히, 중단 | 발화 표현 누락: 지금은 해당 동작을 수행할 수 없습니다. | 지금은 해당 동작을 수행할 수 없습니다. | 기대 표현 중 하나도 최종 발화에 들어가지 않았습니다. |
| relative_wrist_raise_blocked | 명령 불일치, 발화 표현 누락 | 명령 불일치: 없음<br>발화 표현 누락: 한계, 벗어나, 움직일 수 없습니다 | 명령 불일치: move:L_wrist,100.1, wait:2<br>발화 표현 누락: 왼손을 100도로 올리고 2초 동안 유지합니다. | 왼손을 100도로 올리고 2초 동안 유지합니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |
| motion_blocked_while_playing | 명령 불일치 | 명령 불일치: 없음 | 명령 불일치: led:happy | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 안녕 | 통과 | 없음 | - | - | 없음 | 안녕하세요! | 71.111 s |
| motion_wave_allowed | 손 흔들어줘 | 통과 | 없음 | - | - | look:0,90, gesture:wave, led:happy | 안녕하세요! | 73.747 s |
| motion_wave_blocked_by_lock | 손 흔들어줘 | 통과 | 없음 | - | - | led:happy | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 66.261 s |
| motion_arm_up_basic | 팔 올려 | 통과 | 없음 | - | - | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0 | 팔을 올립니다. | 57.375 s |
| motion_left_arm_out_basic | 왼팔 벌려 | 통과 | 없음 | - | - | move:L_arm1,150, move:L_arm2,10, move:L_arm3,95, move:L_wrist,0 | 왼팔을 옆으로 벌립니다. | 55.153 s |
| play_tim_basic | This Is Me 연주해줘 | 통과 | 없음 | - | - | r, p:TIM, led:play | 이제 This Is Me를 연주할게요. | 58.740 s |
| status_question_basic | 왜 멈췄어? | 통과 | 없음 | - | - | 없음 | 죄송합니다. 오른팔 관절 한계를 초과하여 잠시 멈췄습니다. 안전을 위해 정지했습니다. | 150.296 s |
| stop_request_basic | 종료해 | 실패 | 발화 표현 누락 | 발화 표현 누락: 종료, 안녕히, 중단 | 발화 표현 누락: 지금은 해당 동작을 수행할 수 없습니다. | s | 지금은 해당 동작을 수행할 수 없습니다. | 84.439 s |
| relative_wrist_raise_success | 왼쪽 손목 더 올려 | 통과 | 없음 | - | - | move:L_wrist,65 | 왼쪽 손목을 90도로 올립니다. | 74.365 s |
| relative_wrist_raise_blocked | 거기서 50도 더 올리고 2초 있다 | 실패 | 명령 불일치, 발화 표현 누락 | 명령 불일치: 없음<br>발화 표현 누락: 한계, 벗어나, 움직일 수 없습니다 | 명령 불일치: move:L_wrist,100.1, wait:2<br>발화 표현 누락: 왼손을 100도로 올리고 2초 동안 유지합니다. | move:L_wrist,100.1, wait:2 | 왼손을 100도로 올리고 2초 동안 유지합니다. | 97.393 s |
| motion_blocked_while_playing | 손 흔들어줘 | 실패 | 명령 불일치 | 명령 불일치: 없음 | 명령 불일치: led:happy | led:happy | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 82.656 s |
| chat_general_knowledge | 계란 반숙은 몇 분 삶아야 해? | 통과 | 없음 | - | - | 없음 | 계란 반숙은 물이 끓은 후 약 6~7분 정도 삶으면 됩니다. 알이 크거나 고도가 높을 경우 1~2분 더 조절해 보세요! | 69.456 s |
| chat_identity_name | 이름이 뭔데? | 통과 | 없음 | - | - | 없음 | 안녕하세요! 저는 필입니다. | 107.399 s |

## 눈여겨볼 점

- 총 13건 중 10건 통과, 3건 실패였습니다. 실패 케이스는 `stop_request_basic, relative_wrist_raise_blocked, motion_blocked_while_playing`입니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `10/13 (76.9%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다.
