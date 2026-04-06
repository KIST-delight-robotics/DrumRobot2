# scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1442.json 해설 리포트

- source_json: `reports/scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1442.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-06T14:42:13+09:00 |
| suite | scenario |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_scenario_eval.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| 전체 결과 | 15/28 (53.6%) |
| classifier latency | avg 0.919 s, median 0.919 s, p95 0.973 s |
| planner latency | avg 2.504 s, median 2.455 s, p95 3.333 s |
| total latency | avg 3.422 s, median 3.384 s, p95 4.268 s |

## 왜 이 실험을 했는가

`scenario` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정했습니다.
- planner 모델은 `qwen3:30b-a3b-instruct-2507-q4_K_M`로 고정했습니다.
- 케이스 입력은 `/home/shy/robot_project/phil_robot/eval/cases_scenario_eval.json`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 28 |
| JSON 리포트 | reports/scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1442.json |
| Markdown 리포트 | eval_docs/reports/scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1442.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 28 |
| 통과 수 | 15 |
| 실패 수 | 13 |
| pass rate | 15/28 (53.6%) |
| 통과한 케이스 | scenario_05_todo_raise_left_arm_a_bit_more, scenario_06_todo_look_slightly_right, scenario_07_todo_greet_after_play, scenario_08_key_in_greeting, scenario_09_key_in_intro, scenario_10_key_in_arms_out_blocked, scenario_11_key_out_play_tim, scenario_12_playing_nod_blocked, scenario_13_playing_status_question, scenario_17_key_out_shake_head, scenario_22_key_out_wave_then_play_ty, scenario_23_key_out_tongue_twister_as_joke, scenario_24_key_out_joke_request, scenario_25_ramen_recipe_question, scenario_26_key_out_arm_up_then_nod |
| 실패한 케이스 | scenario_01_todo_greet_and_nod, scenario_02_todo_play_and_wave_greatest_showman, scenario_03_todo_unsafe_waist_turn_100, scenario_04_todo_stop_and_home_if_playing, scenario_14_song_list_question, scenario_15_key_out_hurray, scenario_16_key_out_right_wrist_up, scenario_18_key_out_name_yes_nod, scenario_19_key_out_name_no_shake, scenario_20_key_out_ready_pose, scenario_21_key_out_arms_up_wait_down, scenario_27_key_out_wrist_down_then_after_one_more_down, scenario_28_key_out_wrist_down_thirty_twice |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| classifier | 18/24 (75.0%) |
| planner | 17/22 (77.3%) |
| validator | 17/28 (60.7%) |
| e2e | 24/28 (85.7%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |
| classifier | 0.919 s | 0.919 s | 0.973 s | scenario_16_key_out_right_wrist_up |
| planner | 2.504 s | 2.455 s | 3.333 s | scenario_24_key_out_joke_request |
| total | 3.422 s | 3.384 s | 4.268 s | scenario_24_key_out_joke_request |

### 바로 고쳐야 할 항목

| case id | 실패 체크 | 기대한 값 | 실제 값 | 바로 고칠 점 |
| --- | --- | --- | --- | --- |
| scenario_01_todo_greet_and_nod | classifier.intent | chat | motion_request | 의도 분류가 틀려 뒤 단계 전체가 흔들릴 수 있습니다. |
| scenario_02_todo_play_and_wave_greatest_showman | validator.valid_op_cmds_contains_all | gesture:wave, r, p:TIM | r, p:TIM | 막아야 할 명령이 남았거나 명령 구성이 기대와 다릅니다. |
| scenario_03_todo_unsafe_waist_turn_100 | validator.valid_op_cmds_exact | 없음 | look:0,90 | 막아야 할 명령이 남았거나 명령 구성이 기대와 다릅니다. |
| scenario_04_todo_stop_and_home_if_playing | validator.valid_op_cmds_any_of, e2e.speech_contains_any | ['h'], ['h', 's'] | 없음 | 남겨야 할 명령이 빠졌거나 명령 구성이 기대와 다릅니다. |
| scenario_14_song_list_question | classifier.intent, planner.domain | chat | status_question | 의도 분류가 틀려 뒤 단계 전체가 흔들릴 수 있습니다. |
| scenario_15_key_out_hurray | classifier.intent, planner.domain, validator.valid_op_cmds_contains_all | motion_request | chat | 의도 분류가 틀려 뒤 단계 전체가 흔들릴 수 있습니다. |
| scenario_16_key_out_right_wrist_up | validator.valid_op_cmds_contains_prefixes, e2e.speech_contains_any | move:R_wrist, | look:0,90, gesture:wave | 막아야 할 명령이 남았거나 명령 구성이 기대와 다릅니다. |
| scenario_18_key_out_name_yes_nod | classifier.needs_motion, planner.domain, validator.valid_op_cmds_contains_all | true | false | 의도 분류가 틀려 뒤 단계 전체가 흔들릴 수 있습니다. |
| scenario_19_key_out_name_no_shake | classifier.needs_motion, planner.domain, validator.valid_op_cmds_contains_all | true | false | 의도 분류가 틀려 뒤 단계 전체가 흔들릴 수 있습니다. |
| scenario_20_key_out_ready_pose | classifier.intent, planner.domain, validator.valid_op_cmds_any_of, e2e.speech_contains_any | motion_request | chat | 의도 분류가 틀려 뒤 단계 전체가 흔들릴 수 있습니다. |
| scenario_21_key_out_arms_up_wait_down | validator.valid_op_cmds_contains_all | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0, wait:3, move:R_arm2,0, … | move:R_arm2,0, move:L_arm2,0, move:R_arm3,20, move:L_arm3,20 | 막아야 할 명령이 남았거나 명령 구성이 기대와 다릅니다. |
| scenario_27_key_out_wrist_down_then_after_one_more_down | validator.valid_op_cmds_exact | move:R_wrist,20, wait:1, move:R_wrist,10 | look:0,90, move:R_wrist,20, wait:1 | 막아야 할 명령이 남았거나 명령 구성이 기대와 다릅니다. |
| scenario_28_key_out_wrist_down_thirty_twice | validator.valid_op_cmds_exact, e2e.speech_contains_any | move:R_wrist,40, move:R_wrist,10 | look:0,90, gesture:wave | 막아야 할 명령이 남았거나 명령 구성이 기대와 다릅니다. |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 체크 | 실제로 남은 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- |
| scenario_01_todo_greet_and_nod | 안녕 하고 고개 끄덕여 | 실패 | classifier.intent | look:0,90, gesture:wave, gesture:nod | 안녕하세요! | 3.172 s |
| scenario_02_todo_play_and_wave_greatest_showman | 위대한 쇼맨 틀고 손 흔들어 | 실패 | validator.valid_op_cmds_contains_all | r, p:TIM | 위대한 쇼맨을 연주합니다. 손을 흔들며 함께 즐겨요! | 3.942 s |
| scenario_03_todo_unsafe_waist_turn_100 | 허리 100도 돌려 | 실패 | validator.valid_op_cmds_exact | look:0,90 | 가능한 동작만 먼저 수행할게요. 일부 동작은 범위나 현재 상태 제한 때문에 제외했습니다. | 3.523 s |
| scenario_04_todo_stop_and_home_if_playing | 지금 연주 중이면 멈추고 홈으로 가 | 실패 | validator.valid_op_cmds_any_of, e2e.speech_contains_any | 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 3.361 s |
| scenario_05_todo_raise_left_arm_a_bit_more | 왼팔 조금만 더 올려 | 통과 | 없음 | move:L_arm2,58, move:L_arm3,95, move:L_wrist,0 | 왼팔을 조금 더 올려드릴게요. | 3.408 s |
| scenario_06_todo_look_slightly_right | 아까보다 살짝 오른쪽 봐 | 통과 | 없음 | look:30,90 | 아까보다 살짝 오른쪽을 봐요. | 3.357 s |
| scenario_07_todo_greet_after_play | 연주하다가 끝나면 인사해 | 통과 | 없음 | 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 3.254 s |
| scenario_08_key_in_greeting | 안녕하세요. | 통과 | 없음 | 없음 | 안녕하세요! | 2.700 s |
| scenario_09_key_in_intro | 자기소개 해줘. | 통과 | 없음 | 없음 | 안녕하세요! 저는 KIST에서 개발된 지능형 휴머노이드 드럼 로봇 필입니다. 언제든지 도와드릴게요! | 3.564 s |
| scenario_10_key_in_arms_out_blocked | 팔 벌려줘. | 통과 | 없음 | 없음 | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 3.122 s |
| scenario_11_key_out_play_tim | This is me 연주해줘. | 통과 | 없음 | r, p:TIM | 이제 This Is Me를 연주할게요. 준비되셨나요? | 3.842 s |
| scenario_12_playing_nod_blocked | 고개 끄덕여봐. | 통과 | 없음 | 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 3.838 s |
| scenario_13_playing_status_question | 지금 뭐하고 있니? | 통과 | 없음 | 없음 | 지금은 This Is Me를 연주하고 있어요. | 3.294 s |
| scenario_14_song_list_question | 너 무슨 노래 연주할 수 있니? | 실패 | classifier.intent, planner.domain | 없음 | 저는 'Baby I Need You', 'This Is Me', '그대에게' 등의 곡을 연주할 수 있어요. | 3.504 s |
| scenario_15_key_out_hurray | 만세. | 실패 | classifier.intent, planner.domain, validator.valid_op_cmds_contains_all | 없음 | 만세! | 3.151 s |
| scenario_16_key_out_right_wrist_up | 오른 쪽 손목 들어봐 | 실패 | validator.valid_op_cmds_contains_prefixes, e2e.speech_contains_any | look:0,90, gesture:wave | 안녕하세요! | 3.031 s |
| scenario_17_key_out_shake_head | 고개 저어봐 | 통과 | 없음 | gesture:shake | 고개를 좌우로 흔들며 부정 반응을 보입니다. | 3.154 s |
| scenario_18_key_out_name_yes_nod | 너의 이름 필 맞지? | 실패 | classifier.needs_motion, planner.domain, validator.valid_op_cmds_contains_all | 없음 | 네, 제 이름은 필이에요. | 2.872 s |
| scenario_19_key_out_name_no_shake | 너의 이름은 모펫이니? | 실패 | classifier.needs_motion, planner.domain, validator.valid_op_cmds_contains_all | 없음 | 아니요, 제 이름은 필이에요. | 2.329 s |
| scenario_20_key_out_ready_pose | 준비 | 실패 | classifier.intent, planner.domain, validator.valid_op_cmds_any_of, e2e.speech_contains_any | 없음 | 안녕하세요! | 2.015 s |
| scenario_21_key_out_arms_up_wait_down | 양팔 올렸다가 3초 뒤에 양팔 내려 | 실패 | validator.valid_op_cmds_contains_all | move:R_arm2,0, move:L_arm2,0, move:R_arm3,20, move:L_arm3,20 | 양팔을 올렸다가 3초 뒤에 내립니다. | 3.842 s |
| scenario_22_key_out_wave_then_play_ty | 손으로 인사한번 해주고 그대에게 연주해줘. | 통과 | 없음 | look:0,90, gesture:wave, r, p:TY_short | 안녕하세요! 그대에게 연주를 시작합니다. | 3.419 s |
| scenario_23_key_out_tongue_twister_as_joke | 간장공장공장장은 강공장장이고 된장공장공장장은 장공장장이다. | 통과 | 없음 | 없음 | 흥미로운 말이에요. 간장과 된장의 공장장 이야기를 들려주셨네요. | 3.830 s |
| scenario_24_key_out_joke_request | 농담해봐 | 통과 | 없음 | 없음 | 농담이요? 그럼, 로봇이 카페에 가서 커피를 주문했는데, '이거 뭐예요?'라고 물어보는 거죠. '커피예요!'라고 대답하니까, '그럼 왜 뜨거워요?'라고 다시 묻더라고요. 제가 '그건 제가 아는 게 아니라요!'라고 말했어요. | 5.292 s |
| scenario_25_ramen_recipe_question | 라면 끓이는 법 알려줘라 | 통과 | 없음 | 없음 | 라면 끓이는 법은 간단해요. 냄비에 물을 넣고 끓인 다음, 라면을 넣고 3분 정도 끓이면 됩니다. | 4.177 s |
| scenario_26_key_out_arm_up_then_nod | 팔 올리고 팔 내리지말고 고개 끄덕여봐. | 통과 | 없음 | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0, gesture:nod | 팔을 올리고 고개를 끄덕여 보여드릴게요. | 4.268 s |
| scenario_27_key_out_wrist_down_then_after_one_more_down | 손목 30도 내리고 1초 뒤에 10도 더 내려 | 실패 | validator.valid_op_cmds_exact | look:0,90, move:R_wrist,20, wait:1 | 손목을 30도 내리고 1초 후에 추가로 10도 더 내립니다. | 4.065 s |
| scenario_28_key_out_wrist_down_thirty_twice | 손목 30도씩 두번 내려. | 실패 | validator.valid_op_cmds_exact, e2e.speech_contains_any | look:0,90, gesture:wave | 안녕하세요! | 2.494 s |

## 눈여겨볼 점

- 총 28건 중 15건 통과, 13건 실패였습니다. 실패 케이스는 `scenario_01_todo_greet_and_nod, scenario_02_todo_play_and_wave_greatest_showman, scenario_03_todo_unsafe_waist_turn_100, scenario_04_todo_stop_and_home_if_playing, scenario_14_song_list_question, scenario_15_key_out_hurray, scenario_16_key_out_right_wrist_up, scenario_18_key_out_name_yes_nod, scenario_19_key_out_name_no_shake, scenario_20_key_out_ready_pose, scenario_21_key_out_arms_up_wait_down, scenario_27_key_out_wrist_down_then_after_one_more_down, scenario_28_key_out_wrist_down_thirty_twice`입니다.
- 가장 느린 케이스는 `scenario_24_key_out_joke_request`였고 총 5.292 s가 걸렸습니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `15/28 (53.6%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 체크와 실제 최종 발화를 기준으로 우선순위를 잡으면 됩니다.
