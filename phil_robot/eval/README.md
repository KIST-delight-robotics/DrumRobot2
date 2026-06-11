# Phil Robot 평가 파이프라인

## 관련 문서
- planner 후보군의 향후 비교 절차는 [PLANNER_MODEL_BENCHMARK_PLAN_KR.md](/home/shy/robot_project/phil_robot/eval/PLANNER_MODEL_BENCHMARK_PLAN_KR.md)에 정리되어 있습니다.
- round-1 자동 실행 manifest 는 [planner_benchmark_round1_manifest.json](/home/shy/robot_project/phil_robot/eval/planner_benchmark_round1_manifest.json)에 있습니다.

현재 planner benchmark 기본 원칙:
- planner benchmark 는 `JSON production path`만 사용합니다.
- `legacy_str` 대 `json` 비교 스크립트는 과거 형식 비교 실험용이며, planner 모델 후보 선정용 benchmark 에서는 사용하지 않습니다.

현재 기본 case suite:
- `smoke`: 기본 인사/동작/연주/차단 확인
- `scenario` / `scenario_eval`: TODO 예시 + 실제 운영 복합 시나리오 묶음

## 현재 상태 (decision graph refactor 진행 중, 2026-06-10)

런타임 파이프라인을 노드 기반 상태 기계로 분해하는 작업이 진행 중이라, eval 경로는
지금 "그대로 돌아갈 정도"로만 맞춰 둔 전환 상태다. 본격적인 재설계(특히 multi-turn
대화 평가)는 아직 남아 있다. 자세한 단계는 `phil_robot/TODO.md`의 Now 1~3단계 참고.

### 무엇이 바뀌었나
- 런타임 graph(`pipeline/robot_graph.py`)가 한 덩어리 `process_node`에서
  `ingest → classify → state → direct_answer → planner → validator → execute`
  노드 분해 구조로 바뀌었다. 노드 사이에는 `PhilState` 딕셔너리 하나만 굴러다니고,
  예전 `BrainTurnResult` 진단 래퍼는 런타임에서 더 이상 쓰지 않는다.
- 그래서 `run_brain_turn` / `BrainTurnResult`는 런타임 모듈(`pipeline/brain_pipeline.py`)에서
  빠지고 **`phil_robot/eval/brain_probe.py`로 옮겨졌다.** 이 둘은 이제 eval/benchmark 전용
  진단 어댑터다.
- `brain_probe`의 `run_brain_turn`은 런타임 graph와 **같은 step 함수**
  (`build_prefilter_plan` / `classify_step` / `build_direct_answer_plan` / `planner_step`,
  모두 `pipeline/brain_pipeline.py`에 있음)를 호출한다. 즉 "엔진(step 함수)은 하나,
  입구만 둘(런타임 graph / eval probe)"이라 결과가 서로 일치한다.
- `build_classifier_input`이 더 이상 `robot_state`를 받지 않는다(인자 1개: `user_text`).
  classifier system prompt가 상태 요약을 한 번도 참조하지 않던 dead token이라 뺐다.
  상태 판단은 `state_node`와 validator가 책임진다.

### 지금 돌릴 수 있는 것 (single-turn)
- `run_eval.py`의 single-turn suite(`smoke` 등): `brain_probe.run_brain_turn`을 경유해
  한 발화당 한 번 채점. 한 턴 단위 동작이 예전과 같게 나오는지 확인하는 용도.
- planner benchmark / latency isolation: `planner_json_benchmark.py`가 고정 `planner_input`을
  만들어 planner만 반복 측정. 이쪽은 single-call이라 그대로 유효하다.

### 아직 못 하는 것 / 다시 만들어야 하는 것 (multi-turn)
- 새 설계의 되묻기·복구(recovery)는 본질적으로 **여러 턴**에 걸쳐 일어난다.
  예: `팔 돌려줘 → "키 뽑아주세요" → 키 뽑았어 → "몇 도로?" → 200도 → "범위 안내" → 30도 → 실행`.
- `recovery_count`(5회 캡), `pending_intent` 이어주기, 5회째 deterministic 리셋은
  **한 발화만 보는 지금의 케이스 포맷으로는 검증할 수 없다.**
- 그래서 graph를 턴마다 `app.invoke()`로 굴리며 세션 하나를 공유하고 턴별 robot_state를
  주입하는 **multi-turn scenario 러너 + 케이스 포맷**을 따로 만들어야 한다(예정).
- 결론: 지금 eval은 "한 턴짜리 동작이 깨지지 않았는지" 확인까지만 신뢰하고,
  대화·복구 검증은 multi-turn 포맷이 생긴 뒤에 한다.

### 주의
- 이 README 아래의 일부 설명(특히 `설계 메모`의 "production과 동일한 경로" 표현)은
  refactor 이전 기준이라 위 현재 상태 메모가 우선한다.

## scenario eval 설계안 (예정 — graph 구동 multi-turn)

> 아직 구현 안 함. "이렇게 만들겠다"는 설계 기록이다. 3단계(cross-turn recovery) 검증과 함께 만든다.

### 왜 필요한가
- 지금 `run_eval`(smoke)은 `brain_probe.run_brain_turn`(단일 패스)를 쓴다. 이 경로는
  graph 의 **repair 루프(planner↔validator 재호출)도, cross-turn recovery 도 타지 않는다.**
- 그래서 blocked/range/되묻기 케이스는 smoke 에서 첫 시도 결과만 보여 "실패"로 뜨지만,
  런타임(graph)은 정상이다(repair 도메인이 설명/되묻기 생성 — graph 구동으로 확인됨).
- 이 둘을 제대로 검증하려면 **graph 를 턴마다 굴리는** 러너가 필요하다.

### 구동 방식
- `build_phil_graph(...)` 로 만든 app 을 케이스의 turn 마다 `app.invoke({"user_text": ...})` 한다.
- 세션 하나(`SessionContext`)를 케이스 전체에서 공유한다(cross-turn recovery / pending 검증).
- robot_state 는 turn 별로 주입한다: `get_state_fn` 이 그 turn 의 state 를 반환하게 해서
  "키 뽑았어" 같은 세상 변화를 다음 turn 에 반영한다.
- 실제 로봇 소켓은 안 쓴다: bot/executor 는 fake(전송 기록만).
- LLM 은 두 모드:
  - integration: 실제 모델(ollama) — 자연어 품질까지 본다(느림).
  - unit: `call_json_llm` stub — 루프 분기/카운터를 deterministic 하게 본다(빠름).

### 케이스 포맷(초안)
```json
{
  "id": "arm_rotate_recovery",
  "turns": [
    {"user": "팔 돌려줘",   "state": {"is_lock_key_removed": false}, "expect": {"commands_empty": true, "speech_has": "안전 키"}},
    {"user": "키 뽑았어",   "state": {"is_lock_key_removed": true},  "expect": {"commands_empty": true, "speech_has": "몇 도"}},
    {"user": "30도 돌려줘",                                          "expect": {"commands_has": "move:", "speech_has": "30"}}
  ]
}
```
- `turn.state` 는 직전 state 에 덮어쓰는 patch (지정 안 하면 직전 state 유지).
- `expect` 후보: `commands_has` / `commands_empty` / `speech_has` / `repair_attempt` / `recovery_count` / `planner_domain`.

### 커버할 시나리오
- blocked → repair 메시지(명령 0), 다음 턴 해제 → 실행 (cross-turn).
- missing(각도 없음) → 되묻기 → 다음 턴 각도 → 실행.
- range(200도) → 범위 안내 되묻기 → 다음 턴 정상값 → 실행.
- giveup: 5턴 연속 미해결 → deterministic 리셋.
- happy-path: 정상 motion/play/chat/status (repair 0).

### 새 파일(예정)
- `eval/run_scenario.py` (graph 구동 러너)
- `eval/cases_scenario.json` (multi-turn 케이스)
- 기존 `run_eval.py`(single-turn)는 benchmark/단일턴 회귀용으로 유지.

## 목적
이 디렉토리는 `phil_robot`의 Python LLM 제어 스택을 위한 오프라인 평가 파이프라인을 담고 있습니다.

평가 흐름은 실제 로봇 런타임 경로와 의도적으로 분리되어 있습니다.

운영 경로:
- 실제 로봇 상호작용을 위한 경로
- 지연시간과 운영 안정성이 가장 중요함

평가 경로:
- replay, 회귀 테스트, 메트릭 집계를 위한 경로
- 더 느려도 괜찮음
- 더 많은 구조화 데이터를 저장함
- 실제 단계별 출력이 기대값과 일치하는지 비교함

## 평가 레이어
각 평가 케이스는 여러 레이어를 동시에 채점할 수 있습니다.

1. `classifier`
2. `planner`
3. `validator`
4. `e2e`

### Classifier 레이어
검사 항목:
- `intent`
- `needs_motion`

### Planner 레이어
검사 항목:
- `planner_domain`
- 선택된 `skills`
- 생성된 `commands`

### Validator 레이어
검사 항목:
- `valid_commands`
- `rejected_commands`
- 최종 `speech`

### End-to-End 레이어
한 턴 전체 결과를 검사합니다.
- classifier 결과
- planner routing
- validator 결과
- 최종 speech 제약

## 케이스 포맷
케이스 파일은 JSON 배열입니다. 각 케이스는 다음 구조를 가집니다.

```json
{
  "id": "play_tim_basic",
  "tags": ["play", "smoke"],
  "user_text": "This Is Me 연주해줘",
  "robot_state": {
    "state": 0,
    "bpm": 100,
    "is_fixed": true,
    "current_song": "None",
    "progress": "0/1",
    "last_action": "None",
    "is_lock_key_removed": true,
    "error_message": "None",
    "current_angles": {
      "waist": 0.0,
      "L_wrist": 75.0
    }
  },
  "expected": {
    "intent": "play_request",
    "planner_domain": "play",
    "skills_any_of": [["play_tim"], ["ready_pose", "play_tim"]],
    "valid_commands_any_of": [["r", "p:TIM"]],
    "speech_contains_any": ["This Is Me", "연주"]
  }
}
```

## 실행 방법
`~/robot_project`에서 실행:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --suite smoke
```

scenario eval 을 돌리려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --suite scenario
```

짧은 별칭도 사용할 수 있습니다:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --scenario
```

`config.py`를 바꾸지 않고 모델만 임시 override 하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --classifier-model qwen3:4b-instruct-2507-q4_K_M \
  --planner-model qwen3:30b-a3b-instruct-2507-q4_K_M
```

혹은 파일을 직접 지정:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --cases phil_robot/eval/cases_smoke.json
```

scenario eval 파일을 직접 지정하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py --cases phil_robot/eval/cases_scenario_eval.json
```

`~/robot_project/phil_robot/eval` 안에 들어와 있다면:

```bash
python run_eval.py --suite smoke
```

scenario eval 은 이렇게 돌릴 수 있습니다:

```bash
python run_eval.py --scenario
```

## 출력
러너는 다음을 출력합니다.
- 케이스별 pass/fail 요약
- 레이어별 mismatch 상세
- 전체 pass 집계

리포트를 저장하면 JSON과 대응 Markdown이 함께 생성됩니다.

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --report phil_robot/eval/reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json
```

`eval` 폴더 안에 있을 때:

```bash
python run_eval.py --suite smoke --report reports/smoke_report_q3-4b-q4km_q3-30b-a3b-q4km.json
```

위처럼 저장하면 아래 두 파일이 짝으로 생깁니다.

```text
phil_robot/eval/reports/<name>.json
phil_robot/eval/eval_docs/reports/<name>.md
```

표준 파일명 규칙으로 자동 저장하려면 `--save-report`를 사용합니다.

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_eval.py \
  --suite smoke \
  --save-report
```

`eval` 폴더 안에 있을 때:

```bash
python run_eval.py --suite smoke --save-report
```

planner round-1 전체 배치를 순차 실행하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_planner_benchmark.py
```

기본 동작:
- manifest 의 후보 순서를 그대로 따라 prep -> smoke 를 순차 실행합니다.
- classifier 는 케이스당 한 번만 실행해 `classifier_output` 와 `planner_input` 을 고정하고, 각 planner 모델은 같은 JSON fixture 위에서만 비교합니다.
- `config.py`는 수정하지 않고 benchmark 러너 내부에서 classifier/planner 모델을 주입합니다.
- round summary 는 `PLANNER_MODEL_BENCHMARK_ROUND1.json`, `PLANNER_MODEL_BENCHMARK_ROUND1_KR.md`에 저장됩니다.

planner latency isolation benchmark 를 따로 실행하려면:

```bash
/home/shy/miniforge3/envs/drum4/bin/python phil_robot/eval/run_planner_latency_isolation.py \
  --suite smoke \
  --classifier-model qwen3:4b-instruct-2507-q4_K_M \
  --planner-model qwen3:30b-a3b-instruct-2507-q4_K_M \
  --save-report
```

기본 동작:
- classifier 는 케이스당 한 번만 실행해 fixture 를 만들고, 같은 `planner_input` 위에서 planner 만 반복 호출합니다.
- 기본값은 case 당 warm-up `1`회 제외, warm run `3`회 측정입니다.
- 리포트에는 planner input 길이, response 길이, avg / median / p95 latency 와 output variability 가 들어갑니다.

## 리포트 파일명 규칙
자동 생성되는 리포트 파일명은 다음 규칙을 따릅니다.

```text
<suite>_report_<classifier약어>_<planner약어>_<YYYYMMDD_HHMM>.json
```

예:

```text
smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260317_1530.json
```

약어 규칙:
- classifier 약어: 현재 `CLASSIFIER_MODEL` 값에서 생성
- planner 약어: 현재 `PLANNER_MODEL` 값에서 생성
- provider, 파라미터 규모, quant 정보 위주로 압축

동일한 분에 같은 모델 조합으로 다시 저장하면 뒤에 순번을 붙입니다.

```text
smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260317_1530_1.json
smoke_report_q3-4b-q4km_q3-30b-a3b-q4km_20260317_1530_2.json
```

즉:
- 첫 실행은 모델 조합 + 측정 시각이 들어간 이름으로 저장
- 같은 분 안에 같은 모델 조합으로 다시 저장하면 `_1`, `_2` ... 가 자동 증가

## 리포트 메타데이터
새로 저장되는 JSON 리포트에는 다음 메타데이터가 함께 기록됩니다.
- `generated_at`
- `suite`
- `cases_path`
- `classifier_model`
- `planner_model`

이 정보로 나중에 어떤 모델 조합과 어떤 케이스 파일로 측정했는지 추적할 수 있습니다.

대응 Markdown 리포트에는 아래 내용이 함께 정리됩니다.
- 총 몇 건 중 몇 건 통과했는지
- 통과한 케이스 / 실패한 케이스
- 평균 / 중앙값 / p95 지연 시간
- 각 케이스의 실제 최종 발화와 남은 명령

## 기존 리포트 파일에 대한 참고
현재 `reports/` 안에 있는 기존 `smoke_report*.json` 파일 중 일부는
이 규칙을 도입하기 전에 수동으로 저장된 legacy 리포트일 수 있습니다.
새 규칙은 `--save-report`로 생성한 리포트부터 일관되게 적용됩니다.

## 설계 메모
- 이 평가 경로는 실제 classifier / planner 모델을 그대로 호출합니다.
- 실제 로봇 소켓 명령은 전송하지 않습니다.
- 런타임 graph와 동일한 step 함수를 호출하는 `brain_probe.run_brain_turn(...)` 경로를
  재사용해서 현실적인 채점을 합니다. (refactor 이전에는 이 함수가 런타임 모듈에 있었고
  런타임도 직접 호출했지만, 지금은 런타임이 graph 노드 경로를 쓰고 이 함수는 eval 전용입니다.
  위 `현재 상태` 섹션 참고.)
- 예전에 통과하던 한 턴짜리 동작이 이번에도 그대로 통과하는지 확인하는 용도와,
  이후 replay 평가의 기반이 되도록 설계했습니다.
- 대화·복구처럼 여러 턴에 걸친 동작 검증은 multi-turn scenario 포맷이 생긴 뒤에 합니다.
