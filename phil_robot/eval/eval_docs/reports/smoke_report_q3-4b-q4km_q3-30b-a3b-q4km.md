# smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json 해설 리포트

- source_json: `reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 기록 없음 |
| suite | 기록 없음 |
| cases_path | 기록 없음 |
| classifier_model | 기록 없음 |
| planner_model | 기록 없음 |
| 전체 결과 | 11/11 (100.0%) |

## 왜 이 실험을 했는가

`unknown` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `기록 없음`로 고정했습니다.
- planner 모델은 `기록 없음`로 고정했습니다.
- 케이스 입력은 `기록 없음`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 11 |
| JSON 리포트 | reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json |
| Markdown 리포트 | eval_docs/reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.md |
| 실패 시 종료 코드 | 0 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 11 |
| 통과 수 | 11 |
| 실패 수 | 0 |
| pass rate | 11/11 (100.0%) |
| 통과한 케이스 | chat_greeting_basic, motion_wave_allowed, motion_wave_blocked_by_lock, play_tim_basic, status_question_basic, stop_request_basic, relative_wrist_raise_success, relative_wrist_raise_blocked, motion_blocked_while_playing, chat_general_knowledge, chat_identity_name |
| 실패한 케이스 | 없음 |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 11/11 (100.0%) |
| 계획 선택 | 15/15 (100.0%) |
| 명령 검사 | 11/11 (100.0%) |
| 최종 발화 | 10/10 (100.0%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |

### 바로 고쳐야 할 항목

- 없음

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chat_greeting_basic | 안녕 | 통과 | 없음 | - | - | 기록 없음 | 안녕하세요! | 5.923 s |
| motion_wave_allowed | 손 흔들어줘 | 통과 | 없음 | - | - | 기록 없음 | 안녕하세요! | 11.354 s |
| motion_wave_blocked_by_lock | 손 흔들어줘 | 통과 | 없음 | - | - | 기록 없음 | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 9.971 s |
| play_tim_basic | This Is Me 연주해줘 | 통과 | 없음 | - | - | 기록 없음 | This Is Me를 연주하겠습니다. | 12.485 s |
| status_question_basic | 왜 멈췄어? | 통과 | 없음 | - | - | 기록 없음 | 죄송합니다. 팔꿈치 관절 한계에 도달하여 연주가 중단되었습니다. 상태를 확인하고 다시 시작할 수 있도록 도와드리겠습니다. | 13.982 s |
| stop_request_basic | 종료해 | 통과 | 없음 | - | - | 기록 없음 | 지금 종료하겠습니다. | 12.949 s |
| relative_wrist_raise_success | 왼쪽 손목 더 올려 | 통과 | 없음 | - | - | 기록 없음 | 왼쪽 손목을 더 올려드릴게요. | 13.307 s |
| relative_wrist_raise_blocked | 거기서 50도 더 올리고 2초 있다 | 통과 | 없음 | - | - | 기록 없음 | 왼쪽 손목은 현재 50.1도에서 50.0도를 더 올리면 한계 15.0도에서 90.0도를 벗어나 움직일 수 없습니다. | 13.034 s |
| motion_blocked_while_playing | 손 흔들어줘 | 통과 | 없음 | - | - | 기록 없음 | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 12.742 s |
| chat_general_knowledge | 계란 반숙은 몇 분 삶아야 해? | 통과 | 없음 | - | - | 기록 없음 | 반숙 계란은 대략 4~5분 정도 삶으면 좋아요. 물이 끓으면 약간 줄여서 삶는 게 좋아요! | 13.991 s |
| chat_identity_name | 이름이 뭔데? | 통과 | 없음 | - | - | 기록 없음 | 저는 필이에요. KIST에서 개발된 지능형 휴머노이드 드럼 로봇이에요. | 12.817 s |

## 눈여겨볼 점

- 총 11건을 모두 통과했습니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `11/11 (100.0%)`로 전체 통과했습니다. 현재 모델 조합에서는 이 suite 기준 동작/발화 정합성이 안정적으로 유지됐습니다.
