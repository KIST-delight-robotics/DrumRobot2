# Phil Robot — LangGraph 상태 기계 전환 계획

> **구현 상태**: **완료** (2026-04-16). 이 문서는 설계 계획서로 작성됐으며,
> 섹션 9의 모든 파일이 실제로 구현되었다.  
> 구현 세부 사항은 `phil_robot/pipeline/robot_graph.py`,
> `exec_thread.py`, `state_graph.py`를 참고한다.

> **작성 목적**: 이 문서는 `phil_robot`의 동기적(blocking) 실행 루프를
> 비동기 + 상태 기계 구조로 전환하는 작업의 전체 맥락, 설계 결정, 구현 범위를
> 기록한다. 작성자가 아닌 다른 개발자가 이 문서 하나를 읽고 이어서 개발할 수
> 있는 수준으로 작성한다.

---

## 1. 왜 이 작업이 필요한가 — 박사 피드백

### 현상

지금 필(Phil)과 상호작용하면 다음과 같은 경험이 된다.

1. "안녕 반가워" → 손을 흔든다.
2. 손 흔들기가 끝난다.
3. **로봇이 그 자리에서 멈춘다.** (홈 자세로 돌아오지 않음)
4. 다시 말하기 위해 Enter를 누른다.
5. "팔 올려" → 팔을 올린다.
6. 팔을 올리고 있는 **도중에** "다시 손 흔들어"라고 말해도 명령이 **씹힌다**.

박사의 지적 요약: "동작이 끝나면 멈춰버린다. 전환이 너무 로봇 같다."

### 원인 분석

현재 아키텍처의 구조적 문제:

```
phil_brain.py main loop:

[Enter] → record_audio() 3초 blocking
        → transcribe() STT
        → run_brain_turn() LLM 2회
        → execute_validated_plan()   ← time.sleep()으로 Python 스레드 blocking
        → tts.speak()                ← blocking
        → 다시 [Enter] 대기
```

- `execute_validated_plan()` 안의 `wait:` 명령이 `time.sleep()`으로 Python
  메인 스레드를 통째로 블로킹한다.
- TTS도 blocking이다.
- 두 blocking이 모두 끝나야 다시 Enter 입력을 받을 수 있다.
- 실행이 완료된 뒤 홈 자세로 돌아가는 로직이 아예 없다.
- LLM 쪽에는 "지금 어떤 상태에 있는지"를 관리하는 구조가 없어서
  동작 전이 판단을 할 수 없다.

---

## 2. 무엇이 바뀌는가

### 기대 동작 (구현 후)

| 시나리오 | 현재 | 변경 후 |
|----------|------|---------|
| 손 흔들기 끝난 뒤 | 그 자리에 멈춤 | 자동으로 홈 자세(`h`)로 복귀 |
| 팔 올린 채로 새 명령 Enter | 기다렸다가 처리 | 즉시 팔 동작을 중단하고 새 명령 실행 |
| 인사하며 말하는 동시 | 말이 끝난 뒤 제스처 시작 | 말하면서 동시에 제스처 실행 |
| 연주 중 새 명령 | (동일) 가능 | 연주를 멈추지 않고 처리 |

### 바뀌지 않는 것

- classifier / planner / validator / executor 계약 (변경 없음)
- skill 라이브러리 (변경 없음)
- C++ 쪽 로봇 제어기 (변경 없음)
- session.py의 단기 기억 (변경 없음)
- `run_brain_turn()` 시그니처 (변경 없음)

---

## 3. 어떤 도구를 쓰는가 — LangGraph

### LangGraph란

LangGraph는 LLM 애플리케이션에서 상태 기계(state graph)를 선언적으로
정의하기 위한 라이브러리다. 주요 개념:

- **State (TypedDict)**: 그래프의 한 노드에서 다음 노드로 전달되는 데이터 묶음
- **Node (함수)**: State를 받아서 State를 돌려주는 처리 단위
- **Edge**: 노드 사이의 전이 경로. 조건부(`conditional_edges`)로 분기 가능
- **Graph**: 노드와 엣지의 집합. `.compile()` 후 `.invoke(state)`로 실행

### 왜 LangGraph를 쓰는가

1. **명시적 상태 전이**: "지금 로봇이 어떤 상태인지"가 코드 구조에 드러난다.
   `if plan_type == "motion": ...` 분기가 `if-else`로 흩어지지 않고
   그래프 엣지로 표현된다.

2. **확장 경로 확보**: `TODO.md`의 Later 항목들
   (recovery flow, multi-step routing, clarification checkpoint)이
   이 그래프에 새 노드/엣지를 붙이는 방식으로 자연스럽게 연결된다.

3. **장기 로드맵 정렬**: 박사가 요구한 방향이 LangGraph 기반의 에이전트
   아키텍처이므로, 지금 이 구조를 잡아두면 이후 작업에서 재구조화 비용이 없다.

### 현재 설치 상황 (langgraph 설치 불필요)

현재 Python 버전: `3.8.20` (Jetson aarch64)

PyPI에서 aarch64에 설치 가능한 langgraph는 `0.0.8`이 유일한데,
이 버전이 내부적으로 Python 3.9+ 전용 문법(`dict[str, Any]`)을 사용해
Python 3.8에서 import 자체가 TypeError로 실패한다.

따라서 **langgraph 패키지는 설치하지 않으며**, 대신
`phil_robot/pipeline/state_graph.py`라는 경량 호환 구현을 사용한다.

- API 설계를 langgraph와 동일하게 맞춰두었다
  (`StateGraph`, `END`, `add_node`, `add_conditional_edges`, `compile`, `invoke`).
- Python을 3.9+로 올리면 `robot_graph.py` 상단 import 한 줄만 교체하면 된다:
  ```python
  # 현재
  from pipeline.state_graph import StateGraph, END
  # 교체 후
  from langgraph.graph import StateGraph, END
  ```

---

## 4. 상태 기계 설계

### State

```python
# phil_robot/pipeline/robot_graph.py

class PhilState(TypedDict):
    user_text: str          # 이번 턴 사용자 발화
    robot_hw_state: dict    # get_robot_state_snapshot() 스냅샷
    plan_type: str          # "motion" | "play" | "chat" | "stop" | "none"
    speech: str             # 필이 할 말
    commands: list          # 실행할 op_cmd 목록
    play_modifier: dict     # tempo_scale / velocity_delta (play 전용)
    was_interrupted: bool   # 이번 턴이 이전 실행을 중단시켰는가
```

### Nodes

```
process_node  →  execute_node  →  return_home_node
```

| 노드 | 책임 |
|------|------|
| `process_node` | `run_brain_turn()` 호출 → State에 plan_type/speech/commands 채우기 |
| `execute_node` | TTS를 백그라운드 스레드로 시작 + InterruptibleExecutor 시작 |
| `return_home_node` | `plan_type == "motion"`이면 `h` 명령 전송. 그 외에는 pass-through |

### 전이 조건

```
process_node 이후:
  - commands가 있으면              → execute_node
  - commands 없음 (chat-only)     → END

execute_node 이후:
  - plan_type이 motion/posture    → return_home_node
  - plan_type이 play/stop/chat    → END

return_home_node → END
```

### 그래프 다이어그램

```
START
  │
  ▼
process_node
  │
  ├─(commands 있음)────────────────► execute_node
  │                                        │
  └─(chat, commands 없음)──► END           ├─(motion)──► return_home_node ──► END
                                           │
                                           └─(play/stop)──────────────────► END
```

---

## 5. InterruptibleExecutor 설계

### 역할

- 로봇 명령을 **백그라운드 스레드**에서 실행한다.
- `cancel()` 호출 시:
  1. `threading.Event`로 실행 스레드에 중단 신호를 보낸다.
  2. 로봇에 `s` (stop) 명령을 즉시 전송한다.
  3. 실행 스레드가 현재 `wait:` 중이면, 0.05초 단위 loop로 체크하다가 중단한다.

### 파일

`phil_robot/pipeline/exec_thread.py` (신규)

```python
class InterruptibleExecutor:
    def run(self, commands: List[str], on_done: Callable) -> None
    def cancel(self) -> None
    def is_running(self) -> bool
```

- `run()` 호출 시 내부적으로 `threading.Thread`를 시작한다.
- `on_done(cancelled: bool)` 콜백이 완료 또는 취소 후 호출된다.
  - `cancelled=False`: 정상 완료 (home 복귀 트리거)
  - `cancelled=True`: 인터럽트로 종료 (home 복귀 안 함)

---

## 6. Home 복귀 조건 판단

`plan_type`이 어떤 skill/command로 구성되는지에 따라 홈 복귀 여부를 결정한다.

| 동작 유형 | plan_type | 홈 복귀 |
|-----------|-----------|---------|
| gesture (wave, nod, shake, celebrate) | `motion` | ✓ |
| posture (arm_up, arm_down, arms_out 등) | `motion` | ✓ |
| move:* 직접 명령 | `motion` | ✓ |
| look:* 시선 명령 | `motion` | ✗ (시선만 바꾼 것은 유지) |
| p:* 연주 시작 | `play` | ✗ |
| s 정지 | `stop` | ✗ |
| 대화 응답 | `chat` | ✗ |

`plan_type`은 `process_node`에서 classifier result + executed skills의 category를
보고 판단한다. 구체적으로:
- `validated_plan.skills`에 `social` 또는 `posture` category 가 있으면 `"motion"`
- `validated_plan.valid_op_cmds`에 `move:`가 있고 `play`가 없으면 `"motion"`
- `validated_plan.skills`에 `play` category가 있으면 `"play"`
- 그 외 → classifier result의 intent를 그대로 사용

---

## 7. 메인 루프 변경

### 인터럽트 발생 시점

```
[이전 턴: 팔 올리는 중 (executor가 백그라운드에서 wait:2.0 실행 중)]

사용자가 Enter를 누른다

→ if executor.is_running():
       executor.cancel()     # 즉시: 'stop' 로봇 전송 + 스레드 중단
       was_interrupted = True

→ record_audio()             # 새 발화 3초 녹음
→ transcribe()
→ app.invoke(state)          # 새 명령 처리
```

### TTS 동시 실행

```python
# execute_node 내부
tts_thread = Thread(target=tts.speak, args=(speech,), kwargs={"stream": True})
tts_thread.start()
executor.run(commands, on_done=on_done_cb)
# TTS와 동작이 동시에 진행된다
# 말하면서 손을 흔든다
```

### 홈 복귀 타이밍

```python
# on_done_cb (execute_node에서 설정)
def on_done_cb(cancelled: bool):
    if not cancelled and plan_type_for_home_return:
        bot.send_command("h\n")   # 동작이 끝나면 홈으로
```

`on_done_cb`는 executor의 백그라운드 스레드가 완료될 때 호출되므로,
홈 복귀는 **실제 동작이 완료된 시점**에 이루어진다.

---

## 8. TODO.md와의 연결

이 작업이 해결하거나 진전시키는 TODO 항목:

| TODO 항목 | 연결 |
|-----------|------|
| `LangGraph-style state graph 도입 여부 검토` (Later) | **이 작업으로 도입 완료** |
| `session memory 초안` (Next) | 기존 `SessionContext`가 LangGraph State와 공존 |
| `approval / clarification checkpoint` (Next) | 그래프에 `clarification_node` 추가로 자연스럽게 확장 가능 |
| `recovery / resume flow` (Later) | `recovery_node` 추가 경로로 확장 가능 |
| `task planner + dialogue planner 분리` (Next) | `process_node`를 두 노드로 분리하면 됨 |

---

## 9. 구현 파일 목록

| 파일 | 변경 종류 | 내용 |
|------|-----------|------|
| `phil_robot/pipeline/exec_thread.py` | **신규** | `InterruptibleExecutor` |
| `phil_robot/pipeline/robot_graph.py` | **신규** | LangGraph state machine |
| `phil_robot/phil_brain.py` | **수정** | 메인 루프 교체, executor 연결 |
| `phil_robot/environment.yml` | **수정** | `langgraph` 의존성 추가 |
| `phil_robot/TODO.md` | **수정** | LangGraph 항목 Done으로 이동 |
| `log.md` | **수정** | 작업 로그 |

변경 **없음**: `brain_pipeline.py`, `executor.py`, `command_executor.py`,
`validator.py`, `planner.py`, `skills.py`, `session.py`

---

## 10. 전제 조건 / 가정

1. **C++ `s` 명령**: `s` 명령이 현재 실행 중인 동작을 즉시 중단하는지
   확인이 필요하다. `shutdown_system` skill의 `op_cmd`도 `["s"]`이므로
   혼동될 수 있다. C++ 쪽 `AgentSocket` 처리 로직 확인 권장.
   → 만약 `s`가 "stop current motion"이 아니라 "shutdown"이라면,
     별도 명령(예: `stop_motion`)이 필요하다.

2. **홈 복귀 딜레이**: `h` 명령을 보내면 로봇이 즉시 홈으로 가기 시작한다.
   현재 상태 방송(`is_fixed`)을 이용해 동작 완료를 감지하는 것이 이상적이나,
   v1에서는 `on_done_cb`(executor 스레드 완료 시점)에 바로 `h`를 보내는 방식을 쓴다.

3. **look:* 홈 복귀 제외**: 시선 명령은 홈 복귀 대상에서 제외한다.
   예: "오른쪽 봐" → 오른쪽을 보다가 홈으로 돌아오면 어색하다.

4. **online STT와의 관계**: `PLAN.md`에 기록된 online STT 작업은 이 작업과
   독립적이다. 현재 Enter-key 방식 STT에서도 인터럽트는 동작한다
   (Enter를 누르는 순간 executor.cancel() 호출). online STT가 구현되면
   Enter 없이도 인터럽트가 가능해진다.

---

## 11. 한계와 미래 작업

### v1 한계

- Enter를 누르는 시점에만 인터럽트가 발생한다.
  (online STT 전까지는 "말하는 순간" 인터럽트가 아닌 "Enter 누르는 순간" 인터럽트)
- `h` 명령 직후 바로 새 동작이 오면 홈 자세 진행 중에 끊길 수 있다.
  (현재는 허용 — 사용자가 원하는 게 새 동작이기 때문)
- TTS가 재생 중인 동안 새 TTS가 시작되면 겹칠 수 있다.
  (TTS 스레드 join 또는 stop 로직은 추후 추가)

### 다음 단계

1. **online STT 연결** (`PLAN.md`): Enter 없이도 발화 시작 순간에 인터럽트
2. **clarification_node 추가**: 위험/모호 명령에 되묻기 흐름을 그래프 노드로
3. **recovery_node 추가**: 실행 실패 후 안전 복귀
