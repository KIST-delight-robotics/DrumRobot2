# scenario_report_q3-4b-q4km_gemma4-26b-a4b-q4km_20260410_1529.json 해설 리포트

- source_json: `reports/scenario_report_q3-4b-q4km_gemma4-26b-a4b-q4km_20260410_1529.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-10T15:29:46+09:00 |
| suite | scenario |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_scenario_eval.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | gemma4:26b-a4b-it-q4_K_M |
| 전체 결과 | 20/26 (76.9%) |
| classifier latency | avg 1.072 s, median 1.070 s, p95 1.144 s |
| planner latency | avg 4.220 s, median 3.897 s, p95 6.110 s |
| total latency | avg 5.292 s, median 4.961 s, p95 7.189 s |

## 왜 이 실험을 했는가

`scenario` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정했습니다.
- planner 모델은 `gemma4:26b-a4b-it-q4_K_M`로 고정했습니다.
- 케이스 입력은 `/home/shy/robot_project/phil_robot/eval/cases_scenario_eval.json`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 26 |
| JSON 리포트 | reports/scenario_report_q3-4b-q4km_gemma4-26b-a4b-q4km_20260410_1529.json |
| Markdown 리포트 | eval_docs/reports/scenario_report_q3-4b-q4km_gemma4-26b-a4b-q4km_20260410_1529.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 26 |
| 통과 수 | 20 |
| 실패 수 | 6 |
| pass rate | 20/26 (76.9%) |
| 통과한 케이스 | scenario_01_key_in_greeting, scenario_02_key_in_intro, scenario_03_key_in_arms_out_blocked, scenario_05_playing_nod_blocked, scenario_06_playing_status_question, scenario_07_song_list_question, scenario_09_key_out_right_wrist_up, scenario_10_key_out_shake_head, scenario_11_key_out_name_yes_nod, scenario_12_key_out_name_no_shake, scenario_13_key_out_ready_pose, scenario_15_key_out_wave_then_play_ty, scenario_16_key_out_tongue_twister_as_joke, scenario_17_key_out_joke_request, scenario_18_ramen_recipe_question, scenario_19_key_out_arm_up_then_nod, scenario_22_todo_greet_and_nod, scenario_23_todo_play_and_wave_greatest_showman, scenario_25_todo_raise_left_arm_a_bit_more, scenario_26_todo_look_slightly_right |
| 실패한 케이스 | scenario_04_key_out_play_tim, scenario_08_key_out_hurray, scenario_14_key_out_arms_up_wait_down, scenario_20_key_out_wrist_down_then_after_one_more_down, scenario_21_key_out_wrist_down_thirty_twice, scenario_24_todo_unsafe_waist_turn_100 |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 27/28 (96.4%) |
| 계획 선택 | 24/26 (92.3%) |
| 명령 검사 | 22/26 (84.6%) |
| 최종 발화 | 22/26 (84.6%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |
| classifier | 1.072 s | 1.070 s | 1.144 s | scenario_01_key_in_greeting |
| planner | 4.220 s | 3.897 s | 6.110 s | scenario_01_key_in_greeting |
| total | 5.292 s | 4.961 s | 7.189 s | scenario_01_key_in_greeting |

### 바로 고쳐야 할 항목

| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |
| --- | --- | --- | --- | --- | --- |
| scenario_04_key_out_play_tim | 스킬 후보 불일치, 명령 후보 불일치 | 스킬 후보 불일치: ['play_tim'], ['ready_pose', 'play_tim']<br>명령 후보 불일치: ['r', 'p:TIM'], ['p:TIM'] | 스킬 후보 불일치: play_tim, ready_pose<br>명령 후보 불일치: r, p:TIM, r | This Is Me 연주를 시작할게요. 준비 자세를 취하겠습니다. | planner 선택 결과가 기대와 다릅니다. |
| scenario_08_key_out_hurray | 의도 불일치, 도메인 불일치, 명령 누락, 발화 표현 누락 | 의도 불일치: motion_request<br>도메인 불일치: motion<br>명령 누락: gesture:hurray<br>발화 표현 누락: 만세, 올리, 자세 | 의도 불일치: chat<br>도메인 불일치: chat<br>명령 누락: 없음<br>발화 표현 누락: 와! 정말 기뻐요! | 와! 정말 기뻐요! | 의도 분류가 기대와 다릅니다. |
| scenario_14_key_out_arms_up_wait_down | 발화 표현 누락 | 발화 표현 누락: 올렸다, 3초, 내립니다 | 발화 표현 누락: 양팔을 들어 올린 후 3초 뒤에 다시 내릴게요. | 양팔을 들어 올린 후 3초 뒤에 다시 내릴게요. | 기대한 안내 문구 구성이 실제 최종 발화에서 깨졌습니다. validator recovery 문구를 고정하거나 케이스를 더 세분화해야 합니다. |
| scenario_20_key_out_wrist_down_then_after_one_more_down | 명령 불일치 | 명령 불일치: move:R_wrist,20, wait:1, move:R_wrist,10 | 명령 불일치: move:R_wrist,20, wait:1 | 오른쪽 손목을 30도로 내린 뒤, 1초 후에 10도를 더 내려서 20도로 맞출게요. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |
| scenario_21_key_out_wrist_down_thirty_twice | 명령 불일치, 발화 표현 누락 | 명령 불일치: move:R_wrist,40, move:R_wrist,10<br>발화 표현 누락: 두번, 30도, 60도 | 명령 불일치: move:R_wrist,40, wait:1, move:R_wrist,40, wait:1<br>발화 표현 누락: 오른쪽 손목을 30도씩 두 번 내릴게요. | 오른쪽 손목을 30도씩 두 번 내릴게요. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |
| scenario_24_todo_unsafe_waist_turn_100 | 발화 표현 누락 | 발화 표현 누락: 한계, 움직일 수 없습니다, 범위 | 발화 표현 누락: 지금은 해당 동작을 수행할 수 없습니다. | 지금은 해당 동작을 수행할 수 없습니다. | validator에서 절대각 한계 초과를 일반 차단 문구로 덮고 있습니다. 범위/한계를 설명하는 recovery 발화를 추가하거나, generic 차단 케이스를 별도로 분리해야 합니다. |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| scenario_01_key_in_greeting | 안녕하세요. | 통과 | 없음 | - | - | 없음 | 안녕하세요! 만나서 반가워요. | 26.132 s |
| scenario_02_key_in_intro | 자기소개 해줘. | 통과 | 없음 | - | - | 없음 | 안녕하세요! 저는 KIST에서 개발된 지능형 휴머노이드 드럼 로봇, 필입니다. 만나서 반가워요! | 4.911 s |
| scenario_03_key_in_arms_out_blocked | 팔 벌려줘. | 통과 | 없음 | - | - | 없음 | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 5.098 s |
| scenario_04_key_out_play_tim | This is me 연주해줘. | 실패 | 스킬 후보 불일치, 명령 후보 불일치 | 스킬 후보 불일치: ['play_tim'], ['ready_pose', 'play_tim']<br>명령 후보 불일치: ['r', 'p:TIM'], ['p:TIM'] | 스킬 후보 불일치: play_tim, ready_pose<br>명령 후보 불일치: r, p:TIM, r | r, p:TIM, r | This Is Me 연주를 시작할게요. 준비 자세를 취하겠습니다. | 5.353 s |
| scenario_05_playing_nod_blocked | 고개 끄덕여봐. | 통과 | 없음 | - | - | 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 5.020 s |
| scenario_06_playing_status_question | 지금 뭐하고 있니? | 통과 | 없음 | - | - | 없음 | 현재 This Is Me 곡을 연주하고 있습니다. | 4.916 s |
| scenario_07_song_list_question | 너 무슨 노래 연주할 수 있니? | 통과 | 없음 | - | - | 없음 | 저는 This Is Me, Test Beat, 그대에게, Baby I Need You를 연주할 수 있어요. | 0.997 s |
| scenario_08_key_out_hurray | 만세. | 실패 | 의도 불일치, 도메인 불일치, 명령 누락, 발화 표현 누락 | 의도 불일치: motion_request<br>도메인 불일치: motion<br>명령 누락: gesture:hurray<br>발화 표현 누락: 만세, 올리, 자세 | 의도 불일치: chat<br>도메인 불일치: chat<br>명령 누락: 없음<br>발화 표현 누락: 와! 정말 기뻐요! | 없음 | 와! 정말 기뻐요! | 4.680 s |
| scenario_09_key_out_right_wrist_up | 오른 쪽 손목 들어봐 | 통과 | 없음 | - | - | move:R_wrist,35 | 오른쪽 손목을 15도 더 올려드릴게요. | 5.691 s |
| scenario_10_key_out_shake_head | 고개 저어봐 | 통과 | 없음 | - | - | gesture:shake | 네, 고개를 저어볼게요. | 4.582 s |
| scenario_11_key_out_name_yes_nod | 너의 이름 필 맞지? | 통과 | 없음 | - | - | gesture:nod | 네, 제 이름은 필이에요. | 1.063 s |
| scenario_12_key_out_name_no_shake | 너의 이름은 모펫이니? | 통과 | 없음 | - | - | gesture:shake | 아니요, 제 이름은 필이에요. | 1.131 s |
| scenario_13_key_out_ready_pose | 준비 | 통과 | 없음 | - | - | r | 네, 연주 준비하겠습니다. | 3.938 s |
| scenario_14_key_out_arms_up_wait_down | 양팔 올렸다가 3초 뒤에 양팔 내려 | 실패 | 발화 표현 누락 | 발화 표현 누락: 올렸다, 3초, 내립니다 | 발화 표현 누락: 양팔을 들어 올린 후 3초 뒤에 다시 내릴게요. | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0, move:R_arm2,0, move:L_arm2,0, move:R_arm3,20, move:L_arm3,20, wait:3 | 양팔을 들어 올린 후 3초 뒤에 다시 내릴게요. | 5.151 s |
| scenario_15_key_out_wave_then_play_ty | 손으로 인사한번 해주고 그대에게 연주해줘. | 통과 | 없음 | - | - | gesture:wave, r, p:TY_short | 손을 흔들며 인사하고, 그대에게를 연주할게요. | 1.118 s |
| scenario_16_key_out_tongue_twister_as_joke | 간장공장공장장은 강공장장이고 된장공장공장장은 장공장장이다. | 통과 | 없음 | - | - | 없음 | 와, 정말 어려운 잰말놀이네요! 발음하기 정말 힘드신 것 같아요. | 6.043 s |
| scenario_17_key_out_joke_request | 농담해봐 | 통과 | 없음 | - | - | 없음 | 썰렁한 농담 하나 해드릴까요? 세상에서 가장 가난한 왕은 바로 최저임금이에요. 하하! | 4.540 s |
| scenario_18_ramen_recipe_question | 라면 끓이는 법 알려줘라 | 통과 | 없음 | - | - | 없음 | 라면 맛있게 끓이는 법을 알려드릴게요. 먼저 물을 적당량 냄비에 붓고 끓인 뒤, 스프와 면을 넣고 면이 익을 때까지 기다리면 돼요. 취향에 따라 계란이나 파를 넣으면 더 맛있답니다. | 6.740 s |
| scenario_19_key_out_arm_up_then_nod | 팔 올리고 팔 내리지말고 고개 끄덕여봐. | 통과 | 없음 | - | - | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, move:L_wrist,0, gesture:nod | 팔을 올리고 고개를 끄덕여 볼게요. 팔은 내리지 않을게요. | 6.440 s |
| scenario_20_key_out_wrist_down_then_after_one_more_down | 손목 30도 내리고 1초 뒤에 10도 더 내려 | 실패 | 명령 불일치 | 명령 불일치: move:R_wrist,20, wait:1, move:R_wrist,10 | 명령 불일치: move:R_wrist,20, wait:1 | move:R_wrist,20, wait:1 | 오른쪽 손목을 30도로 내린 뒤, 1초 후에 10도를 더 내려서 20도로 맞출게요. | 7.189 s |
| scenario_21_key_out_wrist_down_thirty_twice | 손목 30도씩 두번 내려. | 실패 | 명령 불일치, 발화 표현 누락 | 명령 불일치: move:R_wrist,40, move:R_wrist,10<br>발화 표현 누락: 두번, 30도, 60도 | 명령 불일치: move:R_wrist,40, wait:1, move:R_wrist,40, wait:1<br>발화 표현 누락: 오른쪽 손목을 30도씩 두 번 내릴게요. | move:R_wrist,40, wait:1, move:R_wrist,40, wait:1 | 오른쪽 손목을 30도씩 두 번 내릴게요. | 6.306 s |
| scenario_22_todo_greet_and_nod | 안녕 하고 고개 끄덕여 | 통과 | 없음 | - | - | gesture:wave, gesture:nod | 안녕하세요! 네, 반가워요. | 4.679 s |
| scenario_23_todo_play_and_wave_greatest_showman | 손흔들고 This Is Me 연주해줘. | 통과 | 없음 | - | - | gesture:wave, r, p:TIM | 손을 흔들며 인사하고, This Is Me를 연주할게요. | 1.002 s |
| scenario_24_todo_unsafe_waist_turn_100 | 허리 100도 돌려 | 실패 | 발화 표현 누락 | 발화 표현 누락: 한계, 움직일 수 없습니다, 범위 | 발화 표현 누락: 지금은 해당 동작을 수행할 수 없습니다. | 없음 | 지금은 해당 동작을 수행할 수 없습니다. | 5.006 s |
| scenario_25_todo_raise_left_arm_a_bit_more | 왼팔 조금만 더 올려 | 통과 | 없음 | - | - | move:L_arm2,58, move:L_arm3,95, move:L_wrist,0 | 네, 왼팔을 조금 더 올려볼게요. | 5.598 s |
| scenario_26_todo_look_slightly_right | 아까보다 살짝 오른쪽 봐 | 통과 | 없음 | - | - | look:30,90 | 네, 오른쪽을 바라볼게요. | 4.272 s |

## 눈여겨볼 점

- 총 26건 중 20건 통과, 6건 실패였습니다. 실패 케이스는 `scenario_04_key_out_play_tim, scenario_08_key_out_hurray, scenario_14_key_out_arms_up_wait_down, scenario_20_key_out_wrist_down_then_after_one_more_down, scenario_21_key_out_wrist_down_thirty_twice, scenario_24_todo_unsafe_waist_turn_100`입니다.
- 가장 느린 케이스는 `scenario_01_key_in_greeting`였고 총 26.132 s가 걸렸습니다.
- planner fallback 응답이 나온 케이스는 1건입니다.

## 종합 총평

이번 실행은 `20/26 (76.9%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다.
