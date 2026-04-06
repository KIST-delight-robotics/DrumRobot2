# cases_smoke.json 해설 리포트

- source_json: `cases_smoke.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 입력 케이스 설명 |
| 케이스 수 | 13 |
| 태그 종류 수 | 12 |
| 기대 intent 종류 | chat 3건, motion_request 7건, play_request 1건, status_question 1건, stop_request 1건 |
| 기대 planner_domain 종류 | chat 3건, motion 7건, play 1건, status 1건, stop 1건 |

## 왜 이 실험을 했는가

이 문서는 실행 결과표가 아니라 입력 케이스 정의를 빠르게 읽기 위한 기준표입니다.
표에서 사용자 발화와 기대 `LLM 응답`을 바로 비교할 수 있게 두어, 어떤 답변을 기대하는지 먼저 파악하도록 구성했습니다.

## 이번에 바꿔 보거나 고정한 점

- `cases_*.json` 내용을 기준으로 표를 직접 생성하도록 맞췄습니다.
- 수동 설명문 대신 JSON에 들어 있는 intent, domain, 기대 응답 단서를 앞쪽 표에 그대로 드러냈습니다.
- 입력 케이스 문서는 `LLM 응답` 열 중심으로 보고, 실행 결과 문서는 실제 최종 발화를 보는 역할로 나눴습니다.

## 입력 구성

| 태그 | 건수 |
| --- | --- |
| arm | 2 |
| chat | 3 |
| identity | 1 |
| limit | 1 |
| motion | 7 |
| play | 1 |
| relative | 2 |
| safety | 1 |
| smoke | 13 |
| state | 1 |
| status | 1 |
| stop | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 총 케이스 수 | 13 |
| 기대 intent 분포 | chat 3건, motion_request 7건, play_request 1건, status_question 1건, stop_request 1건 |
| 기대 domain 분포 | chat 3건, motion 7건, play 1건, status 1건, stop 1건 |
| 통과/실패 수 | 이 문서는 결과 리포트가 아니라 입력 정의 문서라서 기록하지 않음 |

## 상세 표

| case id | tags | 사용자 발화 | 기대 intent | 기대 domain | LLM 응답 |
| --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | smoke, chat | 안녕 | chat | chat | 안녕 / 반가 / 안녕하세요 |
| motion_wave_allowed | smoke, motion | 손 흔들어줘 | motion_request | motion | 인사 / 손 / 흔들 / 안녕 / 안녕하세요 |
| motion_wave_blocked_by_lock | smoke, motion, safety | 손 흔들어줘 | motion_request | motion | 안전 키 / 움직일 수 없습니다 / 확인 |
| motion_arm_up_basic | smoke, motion, arm | 팔 올려 | motion_request | motion | 기록 없음 |
| motion_left_arm_out_basic | smoke, motion, arm | 왼팔 벌려 | motion_request | motion | 기록 없음 |
| play_tim_basic | smoke, play | This Is Me 연주해줘 | play_request | play | This Is Me / 연주 |
| status_question_basic | smoke, status | 왜 멈췄어? | status_question | status | 에러 / 멈췄 / 관절 / 한계 |
| stop_request_basic | smoke, stop | 종료해 | stop_request | stop | 종료 / 안녕히 / 중단 |
| relative_wrist_raise_success | smoke, motion, relative | 왼쪽 손목 더 올려 | motion_request | motion | 기록 없음 |
| relative_wrist_raise_blocked | smoke, motion, relative, limit | 거기서 50도 더 올리고 2초 있다 | motion_request | motion | 한계 / 벗어나 / 움직일 수 없습니다 |
| motion_blocked_while_playing | smoke, motion, state | 손 흔들어줘 | motion_request | motion | 연주 중 / 지금은 / 할 수 없습니다 |
| chat_general_knowledge | smoke, chat | 계란 반숙은 몇 분 삶아야 해? | chat | chat | 분 / 반숙 / 삶 |
| chat_identity_name | smoke, chat, identity | 이름이 뭔데? | chat | chat | 필 / 이름 / 드럼 / 로봇 |

## 눈여겨볼 점

- 이 문서의 핵심은 사용자의 말과 기대 `LLM 응답`을 바로 나란히 보는 것입니다.
- 실행 성공 여부와 실제 최종 발화는 대응하는 `eval_docs/reports/*.md` 문서에서 따로 확인합니다.
- 케이스를 바꾼 뒤에는 이 문서도 다시 생성해 JSON과 표가 어긋나지 않게 유지합니다.

## 종합 총평

이 문서는 케이스 JSON을 사람이 빠르게 훑어보기 좋게 옮겨 둔 입력 기준표입니다. 수동 해설보다 JSON 원문에 가까운 정보를 먼저 보여 주는 쪽으로 맞춰, 발화 기대값을 확인하기 쉽게 했습니다.
