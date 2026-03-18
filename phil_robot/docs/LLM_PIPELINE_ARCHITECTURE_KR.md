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
  -> classifier LLM
  -> planner-domain router
  -> planner LLM
  -> planner JSON parse
  -> skill expansion
  -> relative motion resolver
  -> command validator
  -> ValidatedPlan
  -> executor
  -> TTS
  -> robot state feedback
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
  - `led:<emotion>`
  - `move:<motor>,<angle>`
  - `wait:<seconds>`
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
               │ raw_robot_state snapshot
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
│ full adapted state   │    │ classifier state summary │
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
           │                  │ domain router / payload  │  │
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
                                   ┌────────────────────────┐
                                   │      executor.py       │
                                   └────────────┬───────────┘
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

## 모듈별 책임

### 1. Orchestration Layer

파일: [phil_brain.py](/home/shy/robot_project/phil_robot/phil_brain.py)

책임:

- runtime bootstrap
- STT 호출
- state snapshot 획득
- pipeline 호출
- validated plan 실행
- TTS 호출
- 사람이 읽기 좋은 debug 로그 출력

이 파일은 혼합 로직 컨테이너가 아니라 orchestration entrypoint입니다.

### 2. State Adaptation Layer

파일: [state_adapter.py](/home/shy/robot_project/phil_robot/pipeline/state_adapter.py)

책임:

- C++ raw state 정규화
- 내부 song code를 표시용 label로 매핑
- `error_message` -> `error_detail` alias
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
- `needs_dialogue` 추정
- coarse `risk_level` 제공
- motion-bearing intent에 대한 post-parse normalization
- 각도 질문을 `status_question`으로 강제

출력 스키마:

```json
{
  "intent": "chat | motion_request | play_request | status_question | stop_request | unknown",
  "needs_motion": true,
  "needs_dialogue": true,
  "risk_level": "low | medium | high"
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
- planner payload 생성
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
    raw_commands=[...],
    expanded_commands=[...],
    resolved_commands=[...],
    valid_commands=[...],
    rejected_commands=[...],
    warnings=[...],
    speech="...",
    reason="..."
)
```

### 10. Execution Layer

파일:

- [executor.py](/home/shy/robot_project/phil_robot/pipeline/executor.py)
- [command_executor.py](/home/shy/robot_project/phil_robot/pipeline/command_executor.py)

책임:

- validated plan만 소비
- socket client를 통해 실제 로봇 명령 전송
- `wait:<seconds>`를 Python 측 임시 delay primitive로 처리

### 11. Runtime Transport and Feedback Layer

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
4. [state_adapter.py](/home/shy/robot_project/phil_robot/pipeline/state_adapter.py)가 raw runtime state를 정규화한다.
5. [intent_classifier.py](/home/shy/robot_project/phil_robot/pipeline/intent_classifier.py)가 compact classifier payload를 만든다.
6. [llm_interface.py](/home/shy/robot_project/phil_robot/pipeline/llm_interface.py)가 classifier 모델을 호출한다.
7. 발화가 지원되는 deterministic status query이면 [brain_pipeline.py](/home/shy/robot_project/phil_robot/pipeline/brain_pipeline.py)가 현재 상태 스냅샷에서 직접 답한다.
8. 그렇지 않으면 [planner.py](/home/shy/robot_project/phil_robot/pipeline/planner.py)가 `intent`를 planner domain으로 매핑하고 planner payload를 만든다.
9. [llm_interface.py](/home/shy/robot_project/phil_robot/pipeline/llm_interface.py)가 domain-specific prompt로 planner 모델을 호출한다.
10. [planner.py](/home/shy/robot_project/phil_robot/pipeline/planner.py)가 planner JSON을 파싱하고 domain constraint를 적용한다.
11. [validator.py](/home/shy/robot_project/phil_robot/pipeline/validator.py)가 skill을 확장하고 relative motion을 해석한다.
12. [command_validator.py](/home/shy/robot_project/phil_robot/pipeline/command_validator.py)가 low-level command를 검증한다.
13. `ValidatedPlan`이 생성된다.
14. [executor.py](/home/shy/robot_project/phil_robot/pipeline/executor.py)가 검증 통과한 command만 전송한다.
15. [phil_brain.py](/home/shy/robot_project/phil_robot/phil_brain.py)가 최종 speech를 [melo_engine.py](/home/shy/robot_project/phil_robot/runtime/melo_engine.py)로 넘긴다.
16. [phil_client.py](/home/shy/robot_project/phil_robot/runtime/phil_client.py)가 C++에서 갱신된 상태를 받아 다음 턴에 반영한다.
