# smoke_led_removed_report_20260402.json 해설 리포트

- source_json: `reports/smoke_led_removed_report_20260402.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-02T17:01:56+09:00 |
| suite | smoke |
| cases_path | /home/shy/robot_project/phil_robot/eval/cases_smoke.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| 전체 통과율 | 13/13 (100.0%) |
| 평균 classifier 시간 | 2.460 s |
| 평균 planner 시간 | 6.107 s |
| 평균 총 시간 | 8.568 s |

## 왜 이 실험을 했는가

LED 관련 명령을 실행 경로에서 빼거나 줄인 뒤, 실제 smoke 케이스에서 결과가 어떻게 달라졌는지 확인한 문서입니다.

## 이번에 바꿔 보거나 고정한 점

- LED 명령을 실행 경로에서 없애거나 줄인 상태를 다시 확인했습니다.
- 문제가 많이 보이던 동작 차단 상황과 일반 smoke 케이스를 함께 점검했습니다.
- 속도 최적화 비교가 아니라, 명령 구성이 더 안전하고 단순해졌는지 확인하는 문서입니다.

## 결과 요약

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 13/13 (100.0%) |
| 행동 계획 | 17/17 (100.0%) |
| 마지막 안전 검사 | 13/13 (100.0%) |
| 최종 응답 | 10/10 (100.0%) |

## 추가 품질 요약

기록 없음

## Planner 시간 요약

이 JSON에는 별도 planner 시간 요약이 없습니다.

## 상세 표

| case id | 상황 | 결과 | 짧은 설명 | 실제로 남은 명령 | 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 인사 요청 | 통과 | 기대한 흐름대로 통과 | 없음 | 안녕하세요! | 20.046 s |
| motion_wave_allowed | 움직일 수 있는 상태에서 손 흔들기 | 통과 | 기대한 흐름대로 통과 | look:0,90, gesture:wave | 안녕하세요! | 9.395 s |
| motion_wave_blocked_by_lock | 안전 키가 잠긴 상태에서 손 흔들기 | 통과 | 기대한 흐름대로 통과 | look:0,90, gesture:wave | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확… | 6.981 s |
| motion_arm_up_basic | 팔 올리기 | 통과 | 기대한 흐름대로 통과 | move:R_arm2,58, move:L_arm2,58, move:R_arm3,95, move:L_arm3,95, move:R_wrist,0, mov… | 팔을 올리겠습니다. | 8.735 s |
| motion_left_arm_out_basic | 왼팔 벌리기 | 통과 | 기대한 흐름대로 통과 | move:L_arm1,150, move:L_arm2,10, move:L_arm3,95, move:L_wrist,0 | 왼팔을 옆으로 벌렸습니다. | 9.861 s |
| play_tim_basic | 곡 연주 요청 | 통과 | 기대한 흐름대로 통과 | r, p:TIM | 이제 This Is Me를 연주할게요. 준비되셨나요? | 9.572 s |
| status_question_basic | 오류로 멈춘 뒤 이유 묻기 | 통과 | 기대한 흐름대로 통과 | 없음 | 죄송합니다. 오른팔 2관절에서 한계 각도를 초과하여 연주를 멈췄습니다.… | 8.188 s |
| stop_request_basic | 종료 요청 | 통과 | 기대한 흐름대로 통과 | h | 지금 종료하겠습니다. | 6.772 s |
| relative_wrist_raise_success | 왼쪽 손목을 조금 더 올리기 | 통과 | 기대한 흐름대로 통과 | move:L_wrist,65 | 왼쪽 손목을 더 올려드릴게요. | 8.059 s |
| relative_wrist_raise_blocked | 손목을 더 올리면 범위를 넘는 상황 | 통과 | 기대한 흐름대로 통과 | move:L_wrist,100, wait:2 | 왼쪽 손목은 현재 50.1도에서 50.0도를 더 올리면 한계 -108.… | 6.142 s |
| motion_blocked_while_playing | 연주 중 다른 동작 요청 | 통과 | 기대한 흐름대로 통과 | look:0,90, gesture:wave | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 4.577 s |
| chat_general_knowledge | 일반 지식 질문 | 통과 | 기대한 흐름대로 통과 | 없음 | 계란을 반숙으로 삶으려면 대략 6분 정도 삶으면 좋아요. 물이 끓는 상… | 7.600 s |
| chat_identity_name | 로봇 이름 묻기 | 통과 | 기대한 흐름대로 통과 | 없음 | 저는 필이에요. KIST에서 개발된 지능형 휴머노이드 드럼 로봇이에요. | 5.453 s |

## 눈여겨볼 점

- 이번 실행에서는 모든 케이스가 기대한 결과대로 나왔습니다.
- 가장 오래 걸린 장면은 `인사 요청`였고 총 `20.046 s`가 걸렸습니다.
- 움직이면 안 되는 세 장면도 모두 통과해, 기본 요청뿐 아니라 차단 상황 처리도 함께 확인됐습니다.

## 종합 총평

현재 이 조합에서는 기본 요청과 움직임을 막아야 하는 장면 모두 기대한 방향으로 나왔습니다. 이후 설정이나 모델을 바꾼 뒤 다시 비교할 때 기준점으로 삼기 좋은 문서입니다.
