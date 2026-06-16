# Phil Robot 최신 함수 명세서 (Function Specs)

> 이 문서는 파편화된 코드베이스를 하나의 맥락으로 이해하기 위해, 현재(최신) 아키텍처 기준으로 각 파일과 핵심 함수가 정확히 무슨 일을 하는지 매핑한 문서입니다.

---

## 1. 최상위 오케스트레이션 (Orchestration)

### `phil_brain.py`
가장 바깥쪽에서 마이크를 열고, LLM을 호출하고, 로봇에게 명령을 내리는 무한 루프를 돌리는 메인 파일입니다.

- **`main()`**
  - 프로그램의 시작점입니다.
  - `load_runtime()`으로 `RobotClient`(상태 수신 스레드 포함), `MeloTTS`, Whisper STT를 준비합니다.
  - `Executor`(백그라운드 실행기)와 `SessionContext`(단기 기억)를 만듭니다.
  - `build_run_turn(...)`을 호출해 한 턴을 처리하는 `run_turn(user_text)` 클로저를 만듭니다. (예전 `build_phil_graph()`/LangGraph는 폐기됨.)
  - `MicListener().start()`로 마이크 리스너 스레드를 띄운 뒤, 무한 루프에서 확정 발화를 받아 STT → `run_turn(user_text)` → TTS 순으로 처리합니다.
  - TTS는 스레드 비안전이라 항상 **메인 스레드**에서 호출하고, 그 구간은 `listener.set_speaking(True/False)`로 감싸 self-echo를 막습니다.

---

## 2. 한 턴 FSM 및 실행 제어 (Per-turn FSM & Executor)

### `pipeline/robot_fsm.py`
한 턴의 흐름을 langgraph 그래프가 아니라 **imperative FSM**(고정 step 체인 + repair 루프)으로 정의합니다.

```
preprocess → classify → state → direct_answer → (planner ⇄ validator repair) → execute
```

- **`build_run_turn(bot, executor, get_session, get_state_fn, classifier_model, planner_model)`**
  - 각 step 빌더로 클로저를 조립해 `run_turn(user_text)` 함수를 반환합니다.
  - `run_turn`은 `PhilState` 딕셔너리 하나를 step 사이로 굴리며, 진입 시 session의 cross-turn 복구 상태(`recovery_count`/`pending_intent`)를 읽어 giveup/continuation 분기를 정합니다.
- **`make_*_step(...)`** — `preprocess`(prefilter), `classify`(classifier LLM), `fetch_state`(planner 직전 fresh 상태), `direct_answer`(직답 shortcut), `planner`(repair_hint 있으면 repair 도메인), `validator`(`build_validated_plan` + `repair_hint` 추출), `fallback`, `execute`.
- **repair 루프**: `planner → validator`를 돌다 validator가 `repair_hint`를 채우면 사유를 싣고 repair 도메인으로 재호출합니다. `MAX_REPAIR=2`회 안에 못 풀면 `fallback`(명령 폐기 + 안전 문구).
- **`make_execute_step(...)`**: `commands`가 있으면 `executor.exec_cmd(commands, on_done)`로 비동기 전송합니다. `on_done`은 `plan_type == "motion"`일 때만 `home()`을 호출합니다.
- **`home(bot, get_state_fn)`**: **Home Watcher 데몬 스레드**를 띄우고 즉시 반환합니다. 내부 `_watch`는 ① `is_fixed=False`(움직임 시작)를 최대 1.5초 폴링 → 못 보면 홈 복귀 스킵, ② `is_fixed=True`(정지)를 최대 20초 폴링, ③ `h` 전송 순으로 동작합니다. (예전 `_wait_for_fixed_then_home`의 후신.)

### `pipeline/exec_thread.py`
검증된 명령을 백그라운드 스레드에서 순서대로 전송하는 일꾼입니다. wait/cancel 개념은 제거됐습니다.

- **`Executor.exec_cmd(commands, on_done)`**
  - 명령어들을 받아 데몬 스레드(`_run_commands`)를 띄워 하나씩 `bot.send_command()`로 전송하고, 다 보내면 `on_done()`을 호출합니다.
- **`Executor.is_running()`**
  - 백그라운드 전송 스레드가 아직 살아있는지 반환합니다.
- **(제거됨) `cancel()` / `_interruptible_wait()`**
  - `wait:` 지연 명령이 사라지면서 끊을 대상이 없어져 함께 제거됐습니다. 지금 Executor는 "보내고 `on_done`"만 합니다.

---

## 3. LLM 두뇌 파이프라인 (Brain Pipeline)

### `pipeline/brain_pipeline.py`
각 step의 **실제 로직(엔진)**을 모은 파일입니다. `robot_fsm.py`가 이 함수들을 한 턴 FSM으로 엮고, eval(`eval/brain_probe.py`)도 같은 함수를 단일 패스로 호출합니다. (엔진 하나, 입구 둘.)

- **`build_prefilter_plan(user_text)`**
  - LLM 없이 user_text만으로 처리 가능한 결정적 shortcut. pause/resume 키워드, "안녕"+"반가워" 인사 손 흔들기를 잡아 `(classifier_output, planner_output, planner_domain)`을 돌려줍니다. 없으면 `None`.
- **`classify_step(user_text, model, capture_metrics)`**
  - classifier LLM을 호출해 intent를 정하고, 레퍼토리/wave-play 같은 결정적 override를 적용합니다. `(classifier_output, diag)` 반환.
- **`build_direct_answer_plan(user_text, classifier_output, robot_state)`**
  - classifier 결과 + 현재 상태로 planner 없이 답할 수 있는 직답(레퍼토리/이름확인/관절각도/손인사+연주)을 만듭니다. 없으면 `None`.
- **`planner_step(robot_state, user_text, classifier_output, domain, session, model, ..., repair_hint)`**
  - domain별 planner LLM을 호출합니다. `repair_hint`가 있으면 직전 거부 사유를 입력에 실어 repair 도메인으로 계획을 다시 만듭니다.
- **`_detect_play_interrupt(user_text)`**
  - "멈춰"/"계속" 등 pause/resume을 LLM 없이 문자열 매칭으로 빠르게 잡습니다(`build_prefilter_plan` 내부에서 사용).

> 참고: 예전 `run_brain_turn(...)` / `BrainTurnResult`는 런타임에서 빠지고
> `eval/brain_probe.py`로 이주해 eval/benchmark 진단 어댑터로만 남았습니다.

### `pipeline/intent_classifier.py`
- **`build_classifier_input(user_text)`**: 사용자 말만 가벼운 JSON으로 포장합니다. (classifier는 robot_state를 받지 않습니다 — 죽은 토큰이라 제거됨.)
- **`parse_intent_response()`**: LLM이 뱉은 답변에서 `intent`(의도)를 파싱합니다.
- **`normalize_intent_result()`**: LLM이 헷갈린 의도를 안전하게 보정해 줍니다.

### `pipeline/planner.py`
- **`select_planner_domain(classifier_output)`**: 의도에 따라 어떤 프롬프트(chat, play, motion 등)를 쓸지 결정합니다.
- **`parse_plan_response()`**: LLM이 작성한 행동 계획(명령어, 대사)을 JSON에서 추출합니다.

---

## 4. 검열 및 최종 승인 (Validator & Resolver)

### `pipeline/validator.py`
LLM이 만든 계획을 로봇이 이해할 수 있는 최종 형태로 번역하고 위험한 명령을 걸러냅니다.

- **`build_validated_plan(...)`**
  - LLM의 원시 계획(`planner_output`)을 받아서 최종 검열 통과증(`ValidatedPlan`)을 발급합니다.
  - 이 안에서 `skills.py`를 불러 "안녕 인사하기" 같은 추상적 스킬을 `[gesture:wave, wait:2]`처럼 풀어냅니다.
  - 이 안에서 `motion_resolver.py`를 불러 "조금만 더 올려" 같은 상대적 요청을 "50도 위치로 이동" 같은 절대 각도로 계산합니다.
  - 마지막으로 `command_validator.py`를 통해 존재하지 않는 모터나 위험한 각도로 이동하는 걸 막아냅니다.
