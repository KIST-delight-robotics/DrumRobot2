# Phil Robot LLM 파이프라인 아키텍처

## 개요

이 문서는 `phil_robot`의 현재 Python 측 제어 아키텍처를 정리한 것입니다.
초기 음성 루프는 다음과 같은 단일 구조였습니다.

```text
STT -> one LLM call -> parse -> send_command -> TTS
```

현재 구현은 계약, 도메인 라우팅, 상태 적응, 검증, 실행 경계를 갖춘 단계형 파이프라인으로 바뀌었습니다.

현재 상위 단계는 다음과 같습니다.

1. `STT`
2. `런타임 상태 스냅샷`
3. `상태 적응`
4. `의도 분류`
5. `결정적 상태 질의 shortcut`
6. `도메인별 planner`
7. `skill expansion`
8. `상대 모션 해석`
9. `명령 검증`
10. `plan 검증`
11. `실행`
12. `TTS`
13. `상태 피드백`

## 기존 루프와 비교해 무엇이 바뀌었는가

### 기존 아키텍처

```text
Whisper STT
  -> one general-purpose LLM
  -> regex / string parser
  -> send_command(...)
  -> TTS
```

### 현재 아키텍처

```text
Whisper STT
  -> state_adapter
  -> classifier LLM (or prefilter shortcut)
  -> planner-domain router
  -> planner LLM
  -> planner JSON parse
  -> skill expansion
  -> relative motion resolver
  -> command validator
  -> ValidatedPlan
  -> robot_fsm.run_turn (imperative FSM: ... → planner⇄validator repair → execute)
    -> Executor (백그라운드 데몬 스레드, exec_cmd)
    -> TTS (메인 스레드, 동시 실행)
    -> Home Watcher (plan_type == motion 일 때: is_fixed 폴링 후 홈 복귀 h)
  -> robot state feedback
  -> session 갱신 (cross-turn recovery: recovery_count / pending_intent)
```

### 핵심 개선점

- 자유형 제어 문자열 대신 JSON-only LLM 출력
- classifier / planner 분리
- domain-specific planner routing
- skill-first planning
- plan 단위 검증 객체(`ValidatedPlan`)
- 일부 상태 질의에 대한 deterministic 응답
- low-level 런타임 상태와 LLM용 상태 요약 분리
- 안전성 판단을 프롬프트에만 의존하지 않음
- parser 취약성 감소
- 향후 replay / evaluation 삽입 지점 명확화

## 현재 가능한 기능

### 상호작용 기능

- Whisper STT 기반 한국어 음성 입력
- MeloTTS 기반 한국어 음성 출력
- 2단계 LLM 추론
  - 1단계: intent classifier
  - 2단계: planner
- domain-specific planning
  - `chat`
  - `motion`
  - `play`
  - `status`
  - `stop`
  - `generic`
- 안전 상태를 반영한 거절 메시지
- 상태 기반 설명 응답
- 일부 상태 질문에 대한 deterministic direct answer
  - 로봇 이름/정체 질문
  - `왼쪽 손목 몇 도야` 같은 관절 각도 질문

### 제어 기능

- 직접 low-level command 지원
  - `r`
  - `h`
  - `s`
  - `p:<song_code>`
  - `look:pan,tilt`
  - `gesture:<name>`
  - `move:<motor>,<angle>`
  - `pause` (연주 일시정지/재개)
- 상대 모션 해석
  - `올려봐`
  - `내려봐`
  - `50도 더 올려`
  - `거기서 50도 더 올리고 2초 있다`
- 관절 범위 차단
- 로봇 상태 기반 차단
  - safety lock
  - play state
  - error state
  - busy / non-fixed state
- 시퀀스 경고 생성

### Skill-First Planning 기능

Planner 출력은 low-level command만이 아니라 high-level skill도 포함할 수 있습니다.

현재 내장 skill:

- `wave_hi`
- `nod_yes`
- `shake_no`
- `happy_react`
- `celebrate`
- `look_forward`
- `look_left`
- `look_right`
- `look_up`
- `look_down`
- `ready_pose`
- `idle_home`
- `play_tim`
- `play_ty_short`
- `play_bi`
- `play_test_one`
- `shutdown_system`

각 skill은 다음 메타데이터를 가집니다.

- `category`
- `description`
- deterministic low-level `commands`

이 구조 덕분에 planner 출력은 더 재현 가능하고 검증하기 쉬워졌습니다.

## Python LLM 아키텍처 다이어그램

### ASCII Diagram

```text
┌──────────────────────────────┐
│          User Speech         │
└──────────────┬───────────────┘
               │
               v
┌──────────────────────────────┐
│         Whisper STT          │
└──────────────┬───────────────┘
               │ user_text
               v
┌──────────────────────────────┐
│        phil_brain.py         │
│      orchestration layer     │
└──────────────┬───────────────┘
               │ robot_state snapshot
               v
┌──────────────────────────────┐
│       state_adapter.py       │
│  adapt_robot_state()         │
│  build_*_state_summary()     │
└───────┬──────────────┬───────┘
        │              │
        │              └──────────────┐
        │                             │
        v                             v
┌──────────────────────┐    ┌──────────────────────────┐
│ full robot_state     │    │ classifier state summary │
└──────────┬───────────┘    └──────────────┬───────────┘
           │                               │
           │                               v
           │                  ┌──────────────────────────┐
           │                  │   intent_classifier.py   │
           │                  └──────────────┬───────────┘
           │                                 │
           │                                 v
           │                  ┌──────────────────────────┐
           │                  │     llm_interface.py     │
           │                  │     classifier call      │
           │                  └──────────────┬───────────┘
           │                                 │
           │                                 v
           │                  ┌──────────────────────────┐
           │                  │ parse / normalize intent │
           │                  └──────────────┬───────────┘
           │                                 │
           │          deterministic status?  │
           │                  yes ───────────┼───────┐
           │                                 │       │
           │                                 no      v
           │                                 │  ┌──────────────────────┐
           │                                 │  │ direct status reply  │
           │                                 │  └───────────┬──────────┘
           │                                 │              │
           │                                 v              │
           │                  ┌──────────────────────────┐  │
           │                  │       planner.py         │  │
           │                  │ domain router / input JSON │  │
           │                  └──────────────┬───────────┘  │
           │                                 │              │
           │                                 v              │
           │                  ┌──────────────────────────┐  │
           │                  │     llm_interface.py     │  │
           │                  │       planner call       │  │
           │                  └──────────────┬───────────┘  │
           │                                 │              │
           │                                 v              │
           │                  ┌──────────────────────────┐  │
           │                  │ parse plan / constraints │  │
           │                  └──────────────┬───────────┘  │
           │                                 │              │
           │                                 └──────┬───────┘
           │                                        │
           v                                        v
┌──────────────────────────────────────────────────────────┐
│                    validator.py                          │
│        skill expansion + motion resolution +             │
│         command validation + speech finalization         │
└──────────────┬───────────────────────────────┬───────────┘
               │                               │
               v                               v
   ┌──────────────────────┐       ┌─────────────────────────┐
   │      skills.py       │       │  motion_resolver.py     │
   │   expand symbolic    │       │ relative -> absolute    │
   └──────────────────────┘       └─────────────┬───────────┘
                                                │
                                                v
                                   ┌────────────────────────┐
                                   │ command_validator.py   │
                                   │ syntax/range/state     │
                                   └────────────┬───────────┘
                                                │
                                                v
                                   ┌────────────────────────┐
                                   │      ValidatedPlan     │
                                   └────────────┬───────────┘
                                                │
                                                v
                                   ┌────────────────────────────┐
                                   │ robot_fsm.run_turn(execute) │
                                   │ → exec_thread.Executor      │
                                   │   (bg thread) + Home Watcher │
                                   └────────────┬───────────────┘
                                                │
                                                v
                                   ┌────────────────────────┐
                                   │ RobotClient socket I/O │
                                   └────────────┬───────────┘
                                                │
                                                v
                                   ┌────────────────────────┐
                                   │        MeloTTS         │
                                   └────────────────────────┘

C++ robot state broadcast -> runtime/phil_client.py -> thread-safe ROBOT_STATE -> phil_brain.py
```

> ⚠️ **현재 구현 상태에 대한 솔직한 메모 (Patch-work Warning)**
> 현재 `phil_robot/pipeline/brain_pipeline.py` 등 일부 파일은 초기 단일 구조에서 단계형 파이프라인으로 넘어가는 과도기적 산물입니다. 특히 `brain_pipeline.py` 안에는 LLM 지연을 줄이기 위한 각종 지름길(Shortcut, 하드코딩된 예외 처리)이 섞여 있어 코드가 다소 누더기(patch-work)처럼 보일 수 있습니다. 아키텍처(Graph + Executor) 자체는 견고하지만, 내부 접착 코드(Glue code)는 향후 `prefilter.py` 등으로 분리하는 리팩토링이 필요합니다.

자세한 함수 단위 명세서와 각 코드 조각이 무슨 역할을 하는지는 [CURRENT_FUNCTION_SPECS_KR.md](./CURRENT_FUNCTION_SPECS_KR.md)를 참고하세요.

## 모듈별 책임

### 1. Orchestration Layer

파일: [phil_brain.py](/home/shy/robot_project/phil_robot/phil_brain.py)

책임:

- runtime bootstrap (`load_runtime`: RobotClient/TTS/Whisper)
- `MicListener`(VAD)로 발화 단위 audio 수집 → STT 호출
- `build_run_turn(...)` 빌드 및 매 턴 `run_turn(user_text)` 실행 (langgraph `app.invoke`가 아닌 imperative FSM)
- TTS는 메인 스레드에서 호출 (MeloTTS 스레드 비안전), 그 구간은 `set_speaking`으로 마이크 차단
- `SessionContext` 매 턴 갱신 (cross-turn recovery 포함)
- 사람이 읽기 좋은 debug 로그 출력

이 파일은 혼합 로직 컨테이너가 아니라 orchestration entrypoint입니다.

### 2. State Adaptation Layer

파일: [state_adapter.py](/home/shy/robot_project/phil_robot/pipeline/state_adapter.py)

책임:

- C++ 상태 스냅샷 전달
- 원본 `error_message`와 `current_angles`를 그대로 요약에 반영
- low-level Python 제어용 full runtime state 보존
- LLM용 state summary 생성
- joint-angle status query 감지
- 현재 상태 스냅샷 기반 deterministic angle answer 생성

상태는 의도적으로 여러 표현으로 나뉩니다.

#### Full Adapted Runtime State

사용처:

- [brain_pipeline.py](/home/shy/robot_project/phil_robot/pipeline/brain_pipeline.py)
- [motion_resolver.py](/home/shy/robot_project/phil_robot/pipeline/motion_resolver.py)
- [command_validator.py](/home/shy/robot_project/phil_robot/pipeline/command_validator.py)
- [validator.py](/home/shy/robot_project/phil_robot/pipeline/validator.py)

포함 정보:

- `current_angles`
- `last_action`
- full execution context

#### Classifier State Summary

사용처:

- [intent_classifier.py](/home/shy/robot_project/phil_robot/pipeline/intent_classifier.py)

포함 정보:

- mode/state
- busy/can_move
- current song
- current song label
- last action
- error detail

#### Planner State Summary

사용처:

- [planner.py](/home/shy/robot_project/phil_robot/pipeline/planner.py)

포함 정보:

- state
- busy/can_move
- current song
- current song label
- bpm
- progress
- last action
- error detail
- `current_angles`

### 3. Intent Classification Layer

파일: [intent_classifier.py](/home/shy/robot_project/phil_robot/pipeline/intent_classifier.py)

책임:

- 사용자 intent 분류
- `needs_motion` 추정
- motion-bearing intent에 대한 post-parse normalization
- 각도 질문을 `status_question`으로 강제

출력 스키마:

```json
{
  "intent": "chat | motion_request | play_request | status_question | stop_request | unknown",
  "needs_motion": true
}
```

### 4. LLM Interface Layer

파일: [llm_interface.py](/home/shy/robot_project/phil_robot/pipeline/llm_interface.py)

책임:

- Ollama chat 호출 래핑
- JSON output mode 강제
- LLM fallback 처리 집중

### 5. Domain-Specific Planning Layer

파일: [planner.py](/home/shy/robot_project/phil_robot/pipeline/planner.py)

책임:

- `intent`를 planner domain으로 매핑
- domain-specific system prompt 선택
- planner input JSON 생성
- planner JSON 파싱
- post-plan domain constraint 적용

현재 planner domain:

- `chat`
- `motion`
- `play`
- `status`
- `stop`
- `generic`

### 6. Skill Registry Layer

파일: [skills.py](/home/shy/robot_project/phil_robot/pipeline/skills.py)

책임:

- stable high-level behavior macro 유지
- skill category / description 관리
- symbolic action을 deterministic command sequence로 확장
- 연속 중복 command 제거

### 7. Relative Motion Resolution Layer

파일: [motion_resolver.py](/home/shy/robot_project/phil_robot/pipeline/motion_resolver.py)

책임:

- 상대 모션 표현을 절대 motor target으로 변환
- `last_action` 기반 생략된 관절 문맥 추론
- 범위를 넘는 상대 요청을 실행 전 차단
- move가 무효일 때 뒤따르는 `wait`도 함께 제거

### 8. Command Validation Layer

파일: [command_validator.py](/home/shy/robot_project/phil_robot/pipeline/command_validator.py)

책임:

- 문법 검증
- enum 검증
- 범위 검증
- 상태 차단
- legacy normalization
- 시퀀스 경고 생성

### 9. Plan Validation Layer

파일: [validator.py](/home/shy/robot_project/phil_robot/pipeline/validator.py)

책임:

- skill expansion
- relative motion resolution
- low-level command 검증
- warning 병합
- 사용자 facing speech 최종화

실행 계약:

```python
ValidatedPlan(
    skills=[...],
    raw_op_cmds=[...],
    expanded_op_cmds=[...],
    resolved_op_cmds=[...],
    valid_op_cmds=[...],
    rejected_op_cmds=[...],
    warnings=[...],
    speech="...",
    reason="...",
    play_modifier=...,           # 연주 속도/세기 보정값. play 요청 + 빠르기 수식어 있을 때만 설정
    clarification_question="..."  # planner가 되물어야 할 때 설정. 없으면 빈 문자열
)
```

### 10. Execution Layer

파일:

- [exec_thread.py](/home/shy/robot_project/phil_robot/pipeline/exec_thread.py)

책임:

- validated plan에서 추출한 `commands`만 소비
- 백그라운드 데몬 스레드에서 socket client를 통해 실제 로봇 명령 순차 전송
- 전송 완료 시 `on_done()` 호출 (motion이면 Home Watcher 트리거)

> `executor.py` / `command_executor.py`는 제거됐고, `wait:<seconds>` 지연
> primitive도 폐기됐다(validator가 unknown으로 거부). 시간/조건 지연은 향후
> `TaskScheduler` 계층의 책임이다.

### 11. Per-turn FSM Layer (구 LangGraph State Machine)

파일:

- [robot_fsm.py](/home/shy/robot_project/phil_robot/pipeline/robot_fsm.py)
- [exec_thread.py](/home/shy/robot_project/phil_robot/pipeline/exec_thread.py)

책임:

- 한 턴을 `preprocess → classify → state → direct_answer → (planner ⇄ validator repair) → execute` step 체인으로 처리 (`PhilState` 딕셔너리 하나가 step 사이를 굴러다님)
- 턴 내부 repair 루프: validator 거부 시 사유(`repair_hint`)를 repair 도메인 planner로 재호출, `MAX_REPAIR=2` 소진 시 fallback
- `plan_type` 판단 (motion/play/stop/chat)으로 홈 복귀 여부 결정
- `Executor`(`exec_cmd`): 로봇 명령을 백그라운드 데몬 스레드에서 순서대로 전송 후 `on_done()` 호출
- motion 턴이면 `home()`이 **Home Watcher 데몬**을 띄워 `is_fixed` 폴링 후 `h` 전송
- TTS와 로봇 명령 동시 실행 (말하면서 제스처)

> **변경 이력**: 예전엔 langgraph 스타일 선언적 그래프(`robot_graph.py` +
> `state_graph.py` shim, `process → execute → return_home`)였으나, 흐름이
> "고정 체인 + 분기 1~2개"라 graph 추상화 이득이 없어 imperative `run_turn`으로
> 바꿨다. `langgraph`/`state_graph.py`는 제거됐고, `Executor`의 `cancel()`/wait
> 인터럽트도 wait 명령 제거와 함께 사라졌다. 자세한 전환 기록은
> [LANGGRAPH_STATE_MACHINE_KR.md](./LANGGRAPH_STATE_MACHINE_KR.md), 스레드 수명은
> [THREAD_LIFECYCLE_KR.md](./THREAD_LIFECYCLE_KR.md) 참고.

### 12. Session Layer

파일:

- [session.py](/home/shy/robot_project/phil_robot/pipeline/session.py)

책임:

- `SessionContext`: 대화 히스토리, 마지막 관절/시선/연주 상태 단기 기억
- cross-turn recovery 상태 관리 (`recovery_count` / `pending_intent` / `pending_classifier`)
- `update_session()`: actionable 턴인데 실행 명령이 없으면(되묻기/차단/fallback) 미해결로 보고 recovery 진행, 그 외엔 리셋
- `build_session_summary()`: planner input에 포함할 session 요약 생성 (복구 중이면 `pending_intent`를 함께 실어 이번 발화와 합쳐 해석)

> 예전 `pending_clarification_q` + `resolve_clarification_text()`(문자열 결합)는
> 폐기됐다. 되묻기 이어가기는 문자열을 붙이지 않고, classify를 건너뛰고 원래
> classifier/도메인을 재사용하면서 `pending_intent`만 planner에 전달하는 방식으로 바뀌었다.

### 13. Runtime Transport and Feedback Layer

파일:

- [phil_client.py](/home/shy/robot_project/phil_robot/runtime/phil_client.py)
- [melo_engine.py](/home/shy/robot_project/phil_robot/runtime/melo_engine.py)
- [DrumRobot.cpp](/home/shy/robot_project/DrumRobot2/src/DrumRobot.cpp)

책임:

- 비동기 로봇 상태 수신
- thread-safe state snapshot 유지
- deadband 기반 각도 업데이트 병합
- `state == 2`에서 noisy angle spam 억제
- 다음 interaction turn에 runtime feedback 제공
- vendored runtime path를 포함한 MeloTTS 합성 처리

## End-to-End Runtime Flow

1. 사용자가 말한다.
2. Whisper STT가 음성을 텍스트로 변환한다.
3. [phil_brain.py](/home/shy/robot_project/phil_robot/phil_brain.py)가 [phil_client.py](/home/shy/robot_project/phil_robot/runtime/phil_client.py)에서 안정적인 state snapshot을 가져온다.
4. [state_adapter.py](/home/shy/robot_project/phil_robot/pipeline/state_adapter.py)가 runtime state snapshot을 그대로 pipeline에 넘긴다.
5. [intent_classifier.py](/home/shy/robot_project/phil_robot/pipeline/intent_classifier.py)가 compact classifier input JSON을 만든다.
6. [llm_interface.py](/home/shy/robot_project/phil_robot/pipeline/llm_interface.py)가 classifier 모델을 호출한다.
7. 발화가 지원되는 deterministic status query이면 [brain_pipeline.py](/home/shy/robot_project/phil_robot/pipeline/brain_pipeline.py)가 현재 상태 스냅샷에서 직접 답한다.
8. 그렇지 않으면 [planner.py](/home/shy/robot_project/phil_robot/pipeline/planner.py)가 `intent`를 planner domain으로 매핑하고 planner input JSON을 만든다.
9. [llm_interface.py](/home/shy/robot_project/phil_robot/pipeline/llm_interface.py)가 domain-specific prompt로 planner 모델을 호출한다.
10. [planner.py](/home/shy/robot_project/phil_robot/pipeline/planner.py)가 planner JSON을 파싱하고 domain constraint를 적용한다.
11. [validator.py](/home/shy/robot_project/phil_robot/pipeline/validator.py)가 skill을 확장하고 relative motion을 해석한다.
12. [command_validator.py](/home/shy/robot_project/phil_robot/pipeline/command_validator.py)가 low-level command를 검증한다.
13. `ValidatedPlan`이 생성된다.
14. [exec_thread.py](/home/shy/robot_project/phil_robot/pipeline/exec_thread.py)의 `Executor`가 검증 통과한 command만 백그라운드 스레드로 전송한다(motion이면 [robot_fsm.py](/home/shy/robot_project/phil_robot/pipeline/robot_fsm.py)의 Home Watcher가 완료 후 `h`).
15. [phil_brain.py](/home/shy/robot_project/phil_robot/phil_brain.py)가 최종 speech를 [melo_engine.py](/home/shy/robot_project/phil_robot/runtime/melo_engine.py)로 넘긴다.
16. [phil_client.py](/home/shy/robot_project/phil_robot/runtime/phil_client.py)가 C++에서 갱신된 상태를 받아 다음 턴에 반영한다.
