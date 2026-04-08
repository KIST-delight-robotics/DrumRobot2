# targeted_motion_fix_report_20260402.json 해설 리포트

- source_json: `reports/targeted_motion_fix_report_20260402.json`

## 한눈에 보기

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | 실행 결과 리포트 |
| generated_at | 2026-04-02T16:53:50+09:00 |
| suite | targeted_motion_fix_cases |
| cases_path | /tmp/targeted_motion_fix_cases.json |
| classifier_model | qwen3:4b-instruct-2507-q4_K_M |
| planner_model | qwen3:30b-a3b-instruct-2507-q4_K_M |
| 전체 결과 | 4/5 (80.0%) |

## 왜 이 실험을 했는가

`targeted_motion_fix_cases` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.

## 이번에 바꿔 보거나 고정한 점

- classifier 모델은 `qwen3:4b-instruct-2507-q4_K_M`로 고정했습니다.
- planner 모델은 `qwen3:30b-a3b-instruct-2507-q4_K_M`로 고정했습니다.
- 케이스 입력은 `/tmp/targeted_motion_fix_cases.json`를 그대로 사용했습니다.
- `capture_llm_metrics`는 `false` 상태로 실행했습니다.

## 테스트 구성

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 5 |
| JSON 리포트 | reports/targeted_motion_fix_report_20260402.json |
| Markdown 리포트 | eval_docs/reports/targeted_motion_fix_report_20260402.md |
| 실패 시 종료 코드 | 1 |

## 결과 요약

| 항목 | 내용 |
| --- | --- |
| 전체 케이스 수 | 5 |
| 통과 수 | 4 |
| 실패 수 | 1 |
| pass rate | 4/5 (80.0%) |
| 통과한 케이스 | motion_wave_allowed, motion_wave_blocked_by_lock, relative_wrist_raise_success, relative_wrist_raise_blocked |
| 실패한 케이스 | motion_blocked_while_playing |

### 레이어별 통과율

| 단계 | 통과율 |
| --- | --- |
| 의도 분류 | 5/5 (100.0%) |
| 계획 선택 | 5/5 (100.0%) |
| 명령 검사 | 4/5 (80.0%) |
| 최종 발화 | 4/4 (100.0%) |

### 지연 시간 요약

| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |
| --- | --- | --- | --- | --- |

### 바로 고쳐야 할 항목

| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |
| --- | --- | --- | --- | --- | --- |
| motion_blocked_while_playing | 명령 불일치 | 명령 불일치: 없음 | 명령 불일치: led:happy | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 최종 명령이 기대 목록과 정확히 같지 않습니다. |

## 상세 표

| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| motion_wave_allowed | 손 흔들어줘 | 통과 | 없음 | - | - | look:0,90, gesture:wave, led:happy | 안녕하세요! | 22.659 s |
| motion_wave_blocked_by_lock | 손 흔들어줘 | 통과 | 없음 | - | - | led:happy | 아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요. | 4.345 s |
| relative_wrist_raise_success | 왼쪽 손목 더 올려 | 통과 | 없음 | - | - | move:L_wrist,65 | 왼쪽 손목을 더 올려드릴게요. | 4.675 s |
| relative_wrist_raise_blocked | 거기서 50도 더 올리고 2초 있다 | 통과 | 없음 | - | - | 없음 | 왼쪽 손목은 현재 50.1도에서 50.0도를 더 올리면 한계 -108.0도에서 90.0도를 벗어나 움직일 수 없습니다. | 5.731 s |
| motion_blocked_while_playing | 손 흔들어줘 | 실패 | 명령 불일치 | 명령 불일치: 없음 | 명령 불일치: led:happy | led:happy | 지금은 연주 중이라 다른 동작을 할 수 없습니다. | 4.021 s |

## 눈여겨볼 점

- 총 5건 중 4건 통과, 1건 실패였습니다. 실패 케이스는 `motion_blocked_while_playing`입니다.
- planner fallback 응답은 없었습니다.

## 종합 총평

이번 실행은 `4/5 (80.0%)`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다.
