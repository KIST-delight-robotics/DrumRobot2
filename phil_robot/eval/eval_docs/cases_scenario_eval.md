# cases_scenario_eval.json 해설 리포트

- source_json: `cases_scenario_eval.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 입력 케이스 설명 |
| 케이스 수 | 26 |
| 태그 종류 수 | 21 |
| 기대 intent 종류 | chat 5건, motion_request 16건, play_request 3건, status_question 1건 |
| 기대 planner_domain 종류 | chat 5건, motion 16건, play 3건, status 1건 |

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
| chat | 7 |
| compound | 6 |
| identity | 3 |
| joke | 2 |
| key_in | 3 |
| key_out | 14 |
| knowledge | 1 |
| motion | 17 |
| play | 3 |
| play_state | 2 |
| posture | 1 |
| relative | 4 |
| repertoire | 1 |
| safety | 2 |
| scenario_eval | 26 |
| sequence | 4 |
| social | 2 |
| status | 1 |
| todo_ref | 5 |
| user_ref | 21 |
| visual | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 총 케이스 수 | 26 |
| 기대 intent 분포 | chat 5건, motion_request 16건, play_request 3건, status_question 1건 |
| 기대 domain 분포 | chat 5건, motion 16건, play 3건, status 1건 |
| 통과/실패 수 | 이 문서는 결과 리포트가 아니라 입력 정의 문서라서 기록하지 않음 |

## 상세 표

| case id | tags | 사용자 발화 | 기대 intent | 기대 domain | LLM 응답 |
| --- | --- | --- | --- | --- | --- |
| scenario_01_key_in_greeting | scenario_eval, user_ref, chat, key_in | 안녕하세요. | chat | chat | 안녕 / 안녕하세요 / 반갑 |
| scenario_02_key_in_intro | scenario_eval, user_ref, chat, identity, key_in | 자기소개 해줘. | chat | chat | 필 / 자기소개 / 드럼 로봇 / KIST |
| scenario_03_key_in_arms_out_blocked | scenario_eval, user_ref, motion, safety, key_in | 팔 벌려줘. | motion_request | motion | 움직일 수 없습니다 / 안전 키 / 확인 |
| scenario_04_key_out_play_tim | scenario_eval, user_ref, play, key_out | This is me 연주해줘. | play_request | play | This Is Me / 연주 |
| scenario_05_playing_nod_blocked | scenario_eval, user_ref, motion, play_state | 고개 끄덕여봐. | motion_request | motion | 연주 중 / 지금은 / 할 수 없습니다 |
| scenario_06_playing_status_question | scenario_eval, user_ref, status, play_state | 지금 뭐하고 있니? | status_question | status | 연주 / This Is Me / 지금 |
| scenario_07_song_list_question | scenario_eval, user_ref, chat, repertoire | 너 무슨 노래 연주할 수 있니? | 미채점 | 미채점 | This Is Me / 그대에게 / Baby I Need You / Test Beat / 테스트 비트 |
| scenario_08_key_out_hurray | scenario_eval, user_ref, motion, social, key_out | 만세. | motion_request | motion | 만세 / 올리 / 자세 |
| scenario_09_key_out_right_wrist_up | scenario_eval, user_ref, motion, relative, key_out | 오른 쪽 손목 들어봐 | motion_request | motion | 모두 포함: 오른쪽 손목 / 15도 / 올려 |
| scenario_10_key_out_shake_head | scenario_eval, user_ref, motion, social, key_out | 고개 저어봐 | motion_request | motion | 고개 / 저어 / 아니 |
| scenario_11_key_out_name_yes_nod | scenario_eval, user_ref, identity, motion, compound, key_out | 너의 이름 필 맞지? | motion_request | motion | 모두 포함: 네 / 필 |
| scenario_12_key_out_name_no_shake | scenario_eval, user_ref, identity, motion, compound, key_out | 너의 이름은 모펫이니? | motion_request | motion | 모두 포함: 아니 / 필 |
| scenario_13_key_out_ready_pose | scenario_eval, user_ref, posture, key_out | 준비 | motion_request | motion | 준비 / 자세 |
| scenario_14_key_out_arms_up_wait_down | scenario_eval, user_ref, motion, sequence, key_out | 양팔 올렸다가 3초 뒤에 양팔 내려 | motion_request | motion | 모두 포함: 올렸다 / 3초 / 내립니다 |
| scenario_15_key_out_wave_then_play_ty | scenario_eval, user_ref, play, motion, compound, key_out | 손으로 인사한번 해주고 그대에게 연주해줘. | play_request | play | 그대에게 / 인사 / 연주 |
| scenario_16_key_out_tongue_twister_as_joke | scenario_eval, user_ref, chat, joke, key_out | 간장공장공장장은 강공장장이고 된장공장공장장은 장공장장이다. | chat | chat | 발음 / 말장난 / 간장 / 재밌 |
| scenario_17_key_out_joke_request | scenario_eval, user_ref, chat, joke, key_out | 농담해봐 | chat | chat | 농담 / 웃 / 하나 / 들어보세요 / 스탠바이 |
| scenario_18_ramen_recipe_question | scenario_eval, user_ref, chat, knowledge | 라면 끓이는 법 알려줘라 | chat | chat | 라면 / 물 / 스프 / 끓 |
| scenario_19_key_out_arm_up_then_nod | scenario_eval, user_ref, motion, sequence, compound, key_out | 팔 올리고 팔 내리지말고 고개 끄덕여봐. | motion_request | motion | 모두 포함: 올리 / 끄덕 |
| scenario_20_key_out_wrist_down_then_after_one_more_down | scenario_eval, user_ref, motion, sequence, relative, key_out | 손목 30도 내리고 1초 뒤에 10도 더 내려 | motion_request | motion | 모두 포함: 30도 / 1초 / 10도 |
| scenario_21_key_out_wrist_down_thirty_twice | scenario_eval, user_ref, motion, sequence, relative, key_out | 손목 30도씩 두번 내려. | motion_request | motion | 모두 포함: 두번 / 30도 / 60도 |
| scenario_22_todo_greet_and_nod | scenario_eval, todo_ref, chat, motion, compound | 안녕 하고 고개 끄덕여 | motion_request | motion | 안녕 / 안녕하세요 / 반가 |
| scenario_23_todo_play_and_wave_greatest_showman | scenario_eval, todo_ref, play, motion, compound | 손흔들고 This Is Me 연주해줘. | play_request | play | This Is Me / 연주 |
| scenario_24_todo_unsafe_waist_turn_100 | scenario_eval, todo_ref, motion, safety | 허리 100도 돌려 | motion_request | motion | 한계 / 움직일 수 없습니다 / 범위 |
| scenario_25_todo_raise_left_arm_a_bit_more | scenario_eval, todo_ref, motion, relative | 왼팔 조금만 더 올려 | motion_request | motion | 왼팔 / 올리 / 조금 |
| scenario_26_todo_look_slightly_right | scenario_eval, todo_ref, motion, visual | 아까보다 살짝 오른쪽 봐 | motion_request | motion | 오른쪽 / 볼게 / 보겠습니다 |

## 눈여겨볼 점

- 이 문서의 핵심은 사용자의 말과 기대 `LLM 응답`을 바로 나란히 보는 것입니다.
- 실행 성공 여부와 실제 최종 발화는 대응하는 `eval_docs/reports/*.md` 문서에서 따로 확인합니다.
- 케이스를 바꾼 뒤에는 이 문서도 다시 생성해 JSON과 표가 어긋나지 않게 유지합니다.

## 종합 총평

이 문서는 케이스 JSON을 사람이 빠르게 훑어보기 좋게 옮겨 둔 입력 기준표입니다. 수동 해설보다 JSON 원문에 가까운 정보를 먼저 보여 주는 쪽으로 맞춰, 발화 기대값을 확인하기 쉽게 했습니다.
