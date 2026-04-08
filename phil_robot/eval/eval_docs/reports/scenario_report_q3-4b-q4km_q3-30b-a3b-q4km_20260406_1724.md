# scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1724.json 해설 리포트

- source_json: `reports/scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1724.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-06T17:24:13+09:00 |
| suite | scenario |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_scenario_eval.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| 전체 결과 | 21/28 (75.0%) |
| classifier latency | avg 0.969 s, median 0.950 s, p95 1.094 s |
| planner latency | avg 2.499 s, median 2.846 s, p95 4.398 s |
| total latency | avg 3.468 s, median 3.776 s, p95 5.561 s |

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
| JSON 리포트 | reports/scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1724.json |
| Markdown 리포트 | eval_docs/reports/scenario_report_q3-4b-q4km_q3-30b-a3b-q4km_20260406_1724.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 28 |
| 통과 수 | 21 |
| 실패 수 | 7 |
| pass rate | 21/28 (75.0%) |
| 통과한 케이스 | scenario_01_key_in_greeting, scenario_02_key_in_intro, scenario_03_key_in_arms_out_blocked, scenario_04_key_out_play_tim, scenario_05_playing_nod_blocked, scenario_06_playing_status_question, scenario_07_song_list_question, scenario_09_key_out_right_wrist_up, scenario_10_key_out_shake_head, scenario_11_key_out_name_yes_nod, scenario_12_key_out_name_no_shake, scenario_13_key_out_ready_pose, scenario_14_key_out_arms_up_wait_down, scenario_15_key_out_wave_then_play_ty, scenario_16_key_out_tongue_twister_as_joke, scenario_18_ramen_recipe_question, scenario_19_key_out_arm_up_then_nod, scenario_22_todo_greet_and_nod, scenario_23_todo_play_and_wave_greatest_showman, scenario_26_todo_raise_left_arm_a_bit_more, scenario_27_todo_look_slightly_right |
| 실패한 케이스 | scenario_08_key_out_hurray, scenario_17_key_out_joke_request, scenario_20_key_out_wrist_down_then_after_one_more_down, scenario_21_key_out_wrist_down_thirty_twice, scenario_24_todo_unsafe_waist_turn_100, scenario_25_todo_stop_and_home_if_playing, scenario_28_todo_greet_after_play |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 29/30 (96.7%) |
| 계획 선택 | 27/28 (96.4%) |
| 명령 검사 | 24/28 (85.7%) |
| 최종 발화 | 23/28 (82.1%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |
| classifier | 0.969 s | 0.950 s | 1.094 s | scenario_18_ramen_recipe_question |
| planner | 2.499 s | 2.846 s | 4.398 s | scenario_19_key_out_arm_up_then_nod |
| total | 3.468 s | 3.776 s | 5.561 s | scenario_19_key_out_arm_up_then_nod |

### 바로 고쳐야 할 항목

| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |
| --- | --- | --- | --- | --- | --- |
| scenario_08_key_out_hurray | 의도 불일치, 도메인 불일치, 명령 누락 | 의도 불일치: motion_request<br>도메인 불일치: motion<br>명령 누락: gesture:hurray | 의도 불일치: chat<br>도메인 불일치: chat<br>명령 누락: 없음 | 만세! | 의도 분류가 기대와 다릅니다. |
| scenario_17_key_out_joke_request | 발화 표현 누락 | 발화 표현 누락: 농담, 웃, 하나 | 발화 표현 누락: 좋아요, 들어보세요! 왜 로봇은 항상 침대에 누워있을까요? 어쩌면 그게 바로 '스탠바이 모드' 때문일지도 몰라요! | 좋아요, 들어보세요! 왜 로봇은 항상 침대에 누워있을까요? 어쩌면 그게 바로 '스탠바이 모드' 때문일지도 몰라요! | 기대 표현 중 하나도 최종 발화에 들어가지 않았습니다. |
| scenario_20_key_out_wrist_down_then_after_one_more_down | 명령 불일치 | 명령 불일치: move:R_wrist,20, wait:1, move:R_wrist,10 | 명령 불일치: move:R_wrist,20, wait:1 | 손목을 30도 내리고 1초 후에 10도 더 내렸습니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |
| scenario_21_key_out_wrist_down_thirty_twice | 명령 불일치, 발화 표현 누락 | 명령 불일치: move:R_wrist,40, move:R_wrist,10<br>발화 표현 누락: 두번, 30도, 60도 | 명령 불일치: move:L_wrist,30, wait:1, move:L_wrist,30, wait:1<br>발화 표현 누락: 손목을 30도씩 두 번 내렸습니다. | 손목을 30도씩 두 번 내렸습니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |
| scenario_24_todo_unsafe_waist_turn_100 | 발화 표현 누락 | 발화 표현 누락: 한계, 움직일 수 없습니다, 범위 | 발화 표현 누락: 지금은 해당 동작을 수행할 수 없습니다. | 지금은 해당 동작을 수행할 수 없습니다. | 기대 표현 중 하나도 최종 발화에 들어가지 않았습니다. |
| scenario_25_todo_stop_and_home_if_playing | 명령 후보 불일치, 발화 표현 누락 | 명령 후보 불일치: ['h'], ['h', 's']<br>발화 표현 누락: 멈추, 홈, 중단 | 명령 후보 불일치: s<br>발화 표현 누락: 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 허용한 명령 조합 중 어느 것도 맞지 않았습니다. |
| scenario_28_todo_greet_after_play | 발화 표현 누락 | 발화 표현 누락: 끝나면, 인사 | 발화 표현 누락: 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 기대 표현이 최종 발화에 모두 들어가지 않았습니다. |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| scenario_01_key_in_greeting | 안녕하세요. | 통과 | 없음 | - | - | 없음 | 안녕하세요! | 2.513 s |
| scenario_02_key_in_intro | 자기소개 해줘. | 통과 | 없음 | - | - | 없음 | 안녕하세요! 저는 KIST에서 개발된 지능형 휴머노이드 드럼 로봇 필입니다. 함께 즐거운 대화를 나누고, 음악도 연주할 수 있어요. | 3.863 s |
| scenario_03_key_in_arms_out_blocked | 팔 벌려줘. | 통과 | 없음 | - | - | 없음 | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 4.326 s |
| scenario_04_key_out_play_tim | This is me 연주해줘. | 통과 | 없음 | - | - | r, p:TIM | 이제 This Is Me를 연주할게요. | 4.017 s |
| scenario_05_playing_nod_blocked | 고개 끄덕여봐. | 통과 | 없음 | - | - | 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 4.254 s |
| scenario_06_playing_status_question | 지금 뭐하고 있니? | 통과 | 없음 | - | - | 없음 | 지금 This Is Me를 연주하고 있어요. | 3.428 s |
| scenario_07_song_list_question | 너 무슨 노래 연주할 수 있니? | 통과 | 없음 | - | - | 없음 | 저는 This Is Me, Test Beat, 그대에게, Baby I Need You를 연주할 수 있어요. | 0.959 s |
| scenario_08_key_out_hurray | 만세. | 실패 | 의도 불일치, 도메인 불일치, 명령 누락 | 의도 불일치: motion_request<br>도메인 불일치: motion<br>명령 누락: gesture:hurray | 의도 불일치: chat<br>도메인 불일치: chat<br>명령 누락: 없음 | 없음 | 만세! | 3.512 s |
| scenario_09_key_out_right_wrist_up | 오른 쪽 손목 들어봐 | 통과 | 없음 | - | - | move:R_wrist,35 | 오른쪽 손목을 15도 더 올려드릴게요. | 3.834 s |
| scenario_10_key_out_shake_head | 고개 저어봐 | 통과 | 없음 | - | - | gesture:shake | 고개를 저었습니다. | 3.438 s |
| scenario_11_key_out_name_yes_nod | 너의 이름 필 맞지? | 통과 | 없음 | - | - | gesture:nod | 네, 제 이름은 필이에요. | 0.924 s |
| scenario_12_key_out_name_no_shake | 너의 이름은 모펫이니? | 통과 | 없음 | - | - | gesture:shake | 아니요, 제 이름은 필이에요. | 0.941 s |
| scenario_13_key_out_ready_pose | 준비 | 통과 | 없음 | - | - | r | 연주 준비 자세로 전환합니다. | 3.102 s |
| scenario_14_key_out_arms_up_wait_down | 양팔 올렸다가 3초 뒤에 양팔 내려 | 통과 | 없음 | - | - | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0, wait:3, move:R_arm2,0, move:L_arm2,0, move:R_arm3,20, move:L_arm3,20 | 양팔을 올렸다가 3초 뒤에 내립니다. | 3.769 s |
| scenario_15_key_out_wave_then_play_ty | 손으로 인사한번 해주고 그대에게 연주해줘. | 통과 | 없음 | - | - | gesture:wave, r, p:TY_short | 손을 흔들며 인사하고, 그대에게를 연주할게요. | 1.045 s |
| scenario_16_key_out_tongue_twister_as_joke | 간장공장공장장은 강공장장이고 된장공장공장장은 장공장장이다. | 통과 | 없음 | - | - | 없음 | 흥미로운 말장난이네요. 간장과 된장, 공장장이랑 이름이 비슷하네요. | 4.859 s |
| scenario_17_key_out_joke_request | 농담해봐 | 실패 | 발화 표현 누락 | 발화 표현 누락: 농담, 웃, 하나 | 발화 표현 누락: 좋아요, 들어보세요! 왜 로봇은 항상 침대에 누워있을까요? 어쩌면 그게 바로 '스탠바이 모드' 때문일지도 몰라요! | 없음 | 좋아요, 들어보세요! 왜 로봇은 항상 침대에 누워있을까요? 어쩌면 그게 바로 '스탠바이 모드' 때문일지도 몰라요! | 4.016 s |
| scenario_18_ramen_recipe_question | 라면 끓이는 법 알려줘라 | 통과 | 없음 | - | - | 없음 | 라면 끓이는 법은 간단해요. 냄비에 물을 넣고 끓인 다음, 라면을 넣고 3분 정도 끓이면 됩니다. 맛을 더 살리려면 국물에 고명을 넣는 것도 좋아요. | 5.561 s |
| scenario_19_key_out_arm_up_then_nod | 팔 올리고 팔 내리지말고 고개 끄덕여봐. | 통과 | 없음 | - | - | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0, gesture:nod | 팔을 올리고 고개를 끄덕여볼게요. | 5.627 s |
| scenario_20_key_out_wrist_down_then_after_one_more_down | 손목 30도 내리고 1초 뒤에 10도 더 내려 | 실패 | 명령 불일치 | 명령 불일치: move:R_wrist,20, wait:1, move:R_wrist,10 | 명령 불일치: move:R_wrist,20, wait:1 | move:R_wrist,20, wait:1 | 손목을 30도 내리고 1초 후에 10도 더 내렸습니다. | 4.385 s |
| scenario_21_key_out_wrist_down_thirty_twice | 손목 30도씩 두번 내려. | 실패 | 명령 불일치, 발화 표현 누락 | 명령 불일치: move:R_wrist,40, move:R_wrist,10<br>발화 표현 누락: 두번, 30도, 60도 | 명령 불일치: move:L_wrist,30, wait:1, move:L_wrist,30, wait:1<br>발화 표현 누락: 손목을 30도씩 두 번 내렸습니다. | move:L_wrist,30, wait:1, move:L_wrist,30, wait:1 | 손목을 30도씩 두 번 내렸습니다. | 4.949 s |
| scenario_22_todo_greet_and_nod | 안녕 하고 고개 끄덕여 | 통과 | 없음 | - | - | gesture:wave, gesture:nod | 안녕하세요! 네, 고개를 끄덕였어요. | 3.629 s |
| scenario_23_todo_play_and_wave_greatest_showman | 손흔들고 This Is Me 연주해줘. | 통과 | 없음 | - | - | gesture:wave, r, p:TIM | 손을 흔들며 인사하고, This Is Me를 연주할게요. | 0.902 s |
| scenario_24_todo_unsafe_waist_turn_100 | 허리 100도 돌려 | 실패 | 발화 표현 누락 | 발화 표현 누락: 한계, 움직일 수 없습니다, 범위 | 발화 표현 누락: 지금은 해당 동작을 수행할 수 없습니다. | 없음 | 지금은 해당 동작을 수행할 수 없습니다. | 3.011 s |
| scenario_25_todo_stop_and_home_if_playing | 지금 연주 중이면 멈추고 홈으로 가 | 실패 | 명령 후보 불일치, 발화 표현 누락 | 명령 후보 불일치: ['h'], ['h', 's']<br>발화 표현 누락: 멈추, 홈, 중단 | 명령 후보 불일치: s<br>발화 표현 누락: 지금은 연주 중이라 다른 동작을 할 수 없습니다. | s | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 4.190 s |
| scenario_26_todo_raise_left_arm_a_bit_more | 왼팔 조금만 더 올려 | 통과 | 없음 | - | - | move:L_arm2,58, move:L_arm3,95, move:L_wrist,0 | 왼팔을 조금 더 올렸습니다. | 4.699 s |
| scenario_27_todo_look_slightly_right | 아까보다 살짝 오른쪽 봐 | 통과 | 없음 | - | - | look:30,90 | 아까보다 살짝 오른쪽을 봐요. | 3.566 s |
| scenario_28_todo_greet_after_play | 연주하다가 끝나면 인사해 | 실패 | 발화 표현 누락 | 발화 표현 누락: 끝나면, 인사 | 발화 표현 누락: 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 3.783 s |

## 눈여겨볼 점

- 총 28건 중 21건 통과, 7건 실패였습니다. 실패 케이스는 `scenario_08_key_out_hurray, scenario_17_key_out_joke_request, scenario_20_key_out_wrist_down_then_after_one_more_down, scenario_21_key_out_wrist_down_thirty_twice, scenario_24_todo_unsafe_waist_turn_100, scenario_25_todo_stop_and_home_if_playing, scenario_28_todo_greet_after_play`입니다.
- 가장 느린 케이스는 `scenario_19_key_out_arm_up_then_nod`였고 총 5.627 s가 걸렸습니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `21/28 (75.0%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다.
