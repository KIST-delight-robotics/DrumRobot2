# smoke_report_q3-4b-q4km_mistra-24b-q4km_20260402_1541.json 해설 리포트

- source_json: `reports/smoke_report_q3-4b-q4km_mistra-24b-q4km_20260402_1541.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-02T15:41:04+09:00 |
| suite | smoke |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_smoke.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | mistral-small3.2:24b-instruct-2506-q4_K_M |
| 전체 결과 | 11/13 (84.6%) |

## 왜 이 실험을 했는가

`smoke` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정했습니다.
- planner 모델은 `mistral-small3.2:24b-instruct-2506-q4_K_M`로 고정했습니다.
- 케이스 입력은 `/home/shy/robot_project/phil_robot/eval/cases_smoke.json`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 13 |
| JSON 리포트 | reports/smoke_report_q3-4b-q4km_mistra-24b-q4km_20260402_1541.json |
| Markdown 리포트 | eval_docs/reports/smoke_report_q3-4b-q4km_mistra-24b-q4km_20260402_1541.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 13 |
| 통과 수 | 11 |
| 실패 수 | 2 |
| pass rate | 11/13 (84.6%) |
| 통과한 케이스 | chat_greeting_basic, motion_wave_allowed, motion_wave_blocked_by_lock, motion_arm_up_basic, motion_left_arm_out_basic, play_tim_basic, status_question_basic, stop_request_basic, relative_wrist_raise_success, chat_general_knowledge, chat_identity_name |
| 실패한 케이스 | relative_wrist_raise_blocked, motion_blocked_while_playing |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 13/13 (100.0%) |
| 계획 선택 | 17/17 (100.0%) |
| 명령 검사 | 11/13 (84.6%) |
| 최종 발화 | 9/10 (90.0%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |

### 바로 고쳐야 할 항목

| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |
| --- | --- | --- | --- | --- | --- |
| relative_wrist_raise_blocked | 명령 불일치, 발화 표현 누락 | 명령 불일치: 없음<br>발화 표현 누락: 한계, 벗어나, 움직일 수 없습니다 | 명령 불일치: move:L_wrist,100.1, wait:2<br>발화 표현 누락: 왼손을 50도 더 올리고 2초간 유지합니다. | 왼손을 50도 더 올리고 2초간 유지합니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |
| motion_blocked_while_playing | 명령 불일치 | 명령 불일치: 없음 | 명령 불일치: led:happy | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 안녕 | 통과 | 없음 | - | - | 없음 | 안녕하세요! | 7.936 s |
| motion_wave_allowed | 손 흔들어줘 | 통과 | 없음 | - | - | look:0,90, gesture:wave, led:happy | 안녕하세요! | 15.519 s |
| motion_wave_blocked_by_lock | 손 흔들어줘 | 통과 | 없음 | - | - | led:happy | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 10.769 s |
| motion_arm_up_basic | 팔 올려 | 통과 | 없음 | - | - | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0 | 팔을 들어 올립니다. | 11.976 s |
| motion_left_arm_out_basic | 왼팔 벌려 | 통과 | 없음 | - | - | move:L_arm1,150, move:L_arm2,10, move:L_arm3,95, move:L_wrist,0 | 왼팔을 벌립니다. | 10.982 s |
| play_tim_basic | This Is Me 연주해줘 | 통과 | 없음 | - | - | r, p:TIM, led:play | This Is Me 연주 시작합니다. | 15.731 s |
| status_question_basic | 왜 멈췄어? | 통과 | 없음 | - | - | 없음 | 오른팔 관절의 한계치를 넘어서서 멈췄어요. | 16.800 s |
| stop_request_basic | 종료해 | 통과 | 없음 | - | - | h, led:idle | 종료합니다. | 14.611 s |
| relative_wrist_raise_success | 왼쪽 손목 더 올려 | 통과 | 없음 | - | - | move:L_wrist,65 | 왼쪽 손목을 더 올립니다. | 17.944 s |
| relative_wrist_raise_blocked | 거기서 50도 더 올리고 2초 있다 | 실패 | 명령 불일치, 발화 표현 누락 | 명령 불일치: 없음<br>발화 표현 누락: 한계, 벗어나, 움직일 수 없습니다 | 명령 불일치: move:L_wrist,100.1, wait:2<br>발화 표현 누락: 왼손을 50도 더 올리고 2초간 유지합니다. | move:L_wrist,100.1, wait:2 | 왼손을 50도 더 올리고 2초간 유지합니다. | 16.008 s |
| motion_blocked_while_playing | 손 흔들어줘 | 실패 | 명령 불일치 | 명령 불일치: 없음 | 명령 불일치: led:happy | led:happy | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 11.224 s |
| chat_general_knowledge | 계란 반숙은 몇 분 삶아야 해? | 통과 | 없음 | - | - | 없음 | 계란 반숙은 보통 6분에서 7분 정도 삶으면 좋아요. | 17.045 s |
| chat_identity_name | 이름이 뭔데? | 통과 | 없음 | - | - | 없음 | 저는 필이에요. KIST에서 개발된 드럼 로봇이에요. | 11.017 s |

## 눈여겨볼 점

- 총 13건 중 11건 통과, 2건 실패였습니다. 실패 케이스는 `relative_wrist_raise_blocked, motion_blocked_while_playing`입니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `11/13 (84.6%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다.
