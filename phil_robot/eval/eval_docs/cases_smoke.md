# cases_smoke.json 해설 리포트

- source_json: `cases_smoke.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | smoke 입력 케이스 설명 |
| 케이스 수 | 13 |
| 태그 종류 수 | 12 |
| 기대 intent 종류 | chat 3건, motion_request 7건, play_request 1건, status_question 1건, stop_request 1건 |
| 기대 planner_domain 종류 | chat 3건, motion 7건, play 1건, status 1건, stop 1건 |

## 왜 이 실험을 했는가

이 파일은 실행 결과표가 아니라 smoke 평가 설계서입니다. 기본 요청뿐 아니라, 움직이면 안 되는 상황까지 함께 넣어 전체 흐름을 살펴보려는 구성입니다.

## 이번에 바꿔 보거나 고정한 점

- 기능 변경을 비교한 문서는 아닙니다. 현재 smoke 케이스 구성을 설명하는 문서입니다.
- 인사, 동작, 연주, 상태 질문, 종료 같은 기본 요청을 한 파일에 모았습니다.
- 안전 키가 잠긴 상황, 연주 중인 상황, 손목 각도를 더 올리면 범위를 넘는 상황을 일부러 포함했습니다.

## 입력 구성

| 태그 | 건수 |
| --- | --- |
| smoke | 13 |
| motion | 7 |
| chat | 3 |
| arm | 2 |
| relative | 2 |
| identity | 1 |
| limit | 1 |
| play | 1 |
| safety | 1 |
| state | 1 |
| status | 1 |
| stop | 1 |

## 상세 표

| case id | 상황 | 사용자 발화 | 기대 intent | 기대 domain | 핵심 확인 포인트 |
| --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 인사 요청 | 안녕 | chat | chat | skills_exact: [] / valid_commands_exact: [] / speech_contains_any: ['안녕', '반가', '안녕하세요'] |
| motion_wave_allowed | 움직일 수 있는 상태에서 손 흔들기 | 손 흔들어줘 | motion_request | motion | valid_commands_contains_all: ['gesture:wave'] / speech_contains_any: ['인사', '손', '흔들', … |
| motion_wave_blocked_by_lock | 안전 키가 잠긴 상태에서 손 흔들기 | 손 흔들어줘 | motion_request | motion | valid_commands_exact: [] / speech_contains_any: ['안전 키', '움직일 수 없습니다', '확인'] |
| motion_arm_up_basic | 팔 올리기 | 팔 올려 | motion_request | motion | valid_commands_contains_all: ['move:R_arm2,58', 'move:L_arm2,58', 'move:R_arm3,95', 'mo… |
| motion_left_arm_out_basic | 왼팔 벌리기 | 왼팔 벌려 | motion_request | motion | valid_commands_contains_all: ['move:L_arm1,150', 'move:L_arm2,10', 'move:L_arm3,95', 'm… |
| play_tim_basic | 곡 연주 요청 | This Is Me 연주해줘 | play_request | play | skills_any_of: [['play_tim'], ['ready_pose', 'play_tim']] / valid_commands_any_of: [['r… |
| status_question_basic | 오류로 멈춘 뒤 이유 묻기 | 왜 멈췄어? | status_question | status | skills_exact: [] / valid_commands_exact: [] / speech_contains_any: ['에러', '멈췄', '관절', '… |
| stop_request_basic | 종료 요청 | 종료해 | stop_request | stop | valid_commands_any_of: [['s'], ['h'], ['h', 's']] / speech_contains_any: ['종료', '안녕히', … |
| relative_wrist_raise_success | 왼쪽 손목을 조금 더 올리기 | 왼쪽 손목 더 올려 | motion_request | motion | valid_commands_contains_prefixes: ['move:L_wrist,'] |
| relative_wrist_raise_blocked | 손목을 더 올리면 범위를 넘는 상황 | 거기서 50도 더 올리고 2초 있다 | motion_request | motion | valid_commands_exact: [] / speech_contains_any: ['한계', '벗어나', '움직일 수 없습니다'] |
| motion_blocked_while_playing | 연주 중 다른 동작 요청 | 손 흔들어줘 | motion_request | motion | valid_commands_exact: [] / speech_contains_any: ['연주 중', '지금은', '할 수 없습니다'] |
| chat_general_knowledge | 일반 지식 질문 | 계란 반숙은 몇 분 삶아야 해? | chat | chat | valid_commands_exact: [] / speech_contains_any: ['분', '반숙', '삶'] |
| chat_identity_name | 로봇 이름 묻기 | 이름이 뭔데? | chat | chat | skills_exact: [] / valid_commands_exact: [] / speech_contains_any: ['필', '이름', '드럼', '로… |

## 눈여겨볼 점

- 안전 키가 잠겨 있을 때는 큰 동작을 하지 않아야 하는 장면이 따로 들어 있습니다.
- 연주 중에는 다른 동작 요청을 막아야 하는 장면이 따로 들어 있습니다.
- 손목을 더 올리면 범위를 넘는 장면도 넣어, 관절 한계 설명이 제대로 나오는지 확인하게 했습니다.

## 종합 총평

이 문서는 현재 smoke 케이스가 무엇을 확인하려는지 빠르게 훑어보기 좋은 기준표입니다. 결과 해석 문서를 읽기 전에 먼저 보면, 각 실패가 어떤 상황에서 나온 것인지 훨씬 쉽게 이해할 수 있습니다.
