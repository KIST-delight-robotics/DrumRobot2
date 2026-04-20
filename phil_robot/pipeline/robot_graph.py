"""
phil_robot LangGraph 상태 기계.

한 사용자 턴의 흐름을 명시적인 상태 그래프로 표현한다.

  process_node → execute_node → return_home_node → END
                             ↘ (play/chat) → END

노드별 책임:
  process_node    : run_brain_turn() 을 호출하고 State 에 결과를 채운다.
  execute_node    : TTS 를 백그라운드에서 시작하고, InterruptibleExecutor 로
                    로봇 명령을 비동기 실행한다.
  return_home_node: plan_type 이 motion 이면 홈 복귀('h') 명령을 전송한다.
                    on_done 콜백이 실행 완료 후 자동 호출하므로 이 노드는
                    즉시 반환 경로로도 쓰인다.

상태 기계를 도입한 이유:
  - 동작 완료 후 홈 복귀, 실행 중 인터럽트 같은 전이를 명시적으로 표현한다.
  - 향후 clarification_node, recovery_node 추가 시 그래프 엣지만 추가하면 된다.
  - 상태(plan_type, was_interrupted 등)가 그래프를 통해 명확히 전달된다.

설치된 버전: langgraph==0.0.8 (Python 3.8 호환)
"""

import threading
import time
from typing import TYPE_CHECKING, Callable, Dict, List, Optional

# Python 3.8 + Jetson aarch64 환경에서는 경량 호환 구현을 사용한다.
# Python 3.9+ 으로 업그레이드하고 langgraph 를 설치하면 아래 try/except 블록을
# 다음 한 줄로 교체한다:
#   from langgraph.graph import StateGraph, END
try:
    from langgraph.graph import END, StateGraph
except (ImportError, TypeError):
    try:
        from .state_graph import StateGraph, END  # type: ignore[no-redef]
    except ImportError:
        from pipeline.state_graph import StateGraph, END  # type: ignore[no-redef]

from typing import TypedDict

try:
    from .exec_thread import InterruptibleExecutor
    from .skills import SKILL_LIBRARY, get_skill_categories
except ImportError:
    from exec_thread import InterruptibleExecutor  # type: ignore[no-redef]
    from skills import SKILL_LIBRARY, get_skill_categories  # type: ignore[no-redef]

if TYPE_CHECKING:
    from .validator import ValidatedPlan

# ------------------------------------------------------------------
# State
# ------------------------------------------------------------------

class PhilState(TypedDict):
    """
    그래프의 한 노드에서 다음 노드로 전달되는 데이터 묶음.

    phil_brain.py 가 invoke() 를 호출할 때 초기값을 채워 넣는다.
    각 노드는 필요한 키만 갱신하고 나머지는 그대로 전달한다.
    """
    # ── 입력 ──────────────────────────────────────────────────────────
    user_text: str
    robot_hw_state: dict

    # ── process_node 가 채우는 값 ──────────────────────────────────────
    plan_type: str        # "motion" | "play" | "stop" | "chat" | "none"
    speech: str           # 필이 할 말
    commands: List[str]   # valid_op_cmds (실제 전송할 명령 목록)
    play_modifier: Dict   # tempo_scale / velocity_delta (play 전용)
    brain_result: Dict    # BrainTurnResult 를 dict 로 직렬화한 값 (session 갱신용)

    # ── 흐름 제어 ─────────────────────────────────────────────────────
    was_interrupted: bool  # 이번 턴이 이전 실행을 중단시켰는가


# ------------------------------------------------------------------
# plan_type 판단 헬퍼
# ------------------------------------------------------------------

# 이 카테고리 skill 이 포함된 플랜은 실행 후 홈 복귀 대상이다.
_HOME_RETURN_CATEGORIES = {"social", "posture"}

# look:* 같은 시선 명령만 있을 때는 홈 복귀를 하지 않는다.
_LOOK_ONLY_PREFIXES = ("look:",)


def _infer_plan_type(validated_plan: "ValidatedPlan", classifier_intent: str) -> str:
    """
    ValidatedPlan 과 classifier intent 를 보고 plan_type 을 결정한다.

    반환값:
      "motion" - 홈 복귀가 필요한 gesture/posture/move 계열
      "play"   - 연주 시작 (홈 복귀 없음)
      "stop"   - 정지 명령 (홈 복귀 없음)
      "chat"   - 대화 응답만 (명령 없음)
      "none"   - 명령도 없고 의미 있는 분류도 없음
    """
    # skill 기반 판단
    skill_names = list(getattr(validated_plan, "skills", []) or [])
    if skill_names:
        categories = get_skill_categories(skill_names)
        if "play" in categories:
            return "play"
        if _HOME_RETURN_CATEGORIES & categories:
            return "motion"

    # 직접 op_cmd 기반 판단
    cmds = list(getattr(validated_plan, "valid_op_cmds", []) or [])
    if not cmds:
        return "chat" if classifier_intent in ("chat", "status_question") else "none"

    has_play = any(c.startswith("p:") or c == "r" for c in cmds)
    has_stop = any(c == "s" for c in cmds)
    has_move = any(c.startswith("move:") or c.startswith("gesture:") for c in cmds)
    has_look_only = all(c.startswith(_LOOK_ONLY_PREFIXES) or c.startswith("wait:") for c in cmds)

    if has_play:
        return "play"
    if has_stop:
        return "stop"
    if has_move:
        return "motion"
    if has_look_only:
        return "none"   # 시선만 바꾼 것은 홈 복귀 제외

    return classifier_intent if classifier_intent in ("chat", "stop") else "none"


def _extract_commands(validated_plan: "ValidatedPlan") -> List[str]:
    """play_modifier 를 포함한 전체 전송 명령 목록을 만든다."""
    cmds: List[str] = []

    modifier = getattr(validated_plan, "play_modifier", None)
    if modifier is not None:
        if getattr(modifier, "tempo_scale", 1.0) != 1.0:
            cmds.append(f"tempo_scale:{modifier.tempo_scale:.2f}")
        if getattr(modifier, "velocity_delta", 0) != 0:
            cmds.append(f"velocity_delta:{modifier.velocity_delta}")

    cmds.extend(list(getattr(validated_plan, "valid_op_cmds", []) or []))
    return cmds


# ------------------------------------------------------------------
# Node 빌더 — 클로저로 외부 의존성(bot, tts, executor, session 등)을 주입한다.
# ------------------------------------------------------------------

def make_process_node(bot, tts, stt_model, executor, get_session, run_brain_turn_fn,
                      classifier_model: str, planner_model: str):
    """
    process_node 함수를 생성해 반환한다.

    인터럽트 감지(executor.is_running() → cancel)도 여기서 처리한다.
    """
    def process_node(state: PhilState) -> PhilState:
        was_interrupted = False

        # ── 이전 실행 중이면 즉시 중단 ──────────────────────────────────
        if executor.is_running():
            print("[Graph] 이전 실행 중단 (인터럽트)")
            executor.cancel()
            was_interrupted = True

        # ── LLM 파이프라인 실행 ──────────────────────────────────────────
        session = get_session()
        brain_result = run_brain_turn_fn(
            user_text=state["user_text"],
            raw_robot_state=state["robot_hw_state"],
            classifier_model_name=classifier_model,
            planner_model_name=planner_model,
            capture_metrics=True,
            session=session,
        )

        validated = brain_result.validated_plan
        classifier_intent = brain_result.classifier_result.get("intent", "none")
        plan_type = _infer_plan_type(validated, classifier_intent)
        commands = _extract_commands(validated)

        # play_modifier 는 dict 로 변환해 저장한다
        modifier = getattr(validated, "play_modifier", None)
        play_modifier_dict: Dict = {}
        if modifier is not None:
            play_modifier_dict = {
                "tempo_scale": getattr(modifier, "tempo_scale", 1.0),
                "velocity_delta": getattr(modifier, "velocity_delta", 0),
            }

        return {
            **state,
            "plan_type": plan_type,
            "speech": validated.speech or "",
            "commands": commands,
            "play_modifier": play_modifier_dict,
            "brain_result": {"_obj": brain_result},   # executor 외부에서 session 갱신용
            "was_interrupted": was_interrupted,
        }

    return process_node


def _wait_for_fixed_then_home(bot, get_state_fn: Callable) -> None:
    """
    백그라운드 스레드에서 로봇이 실제로 정지(is_fixed=True)할 때까지
    기다린 뒤 홈 복귀 명령('h')을 전송한다.

    흐름:
    1. 로봇이 움직이기 시작할 때까지 대기 (is_fixed=False, 최대 1.5초)
       - 움직임이 한 번도 감지되지 않으면 홈 복귀를 건너뜀.
         C++ allMotorsUnConected=true 상태이면 is_fixed 가 항상 true 여서
         즉시 h 가 발송될 수 있다. C++ 쪽 fix 이후에는 이 guard 만으로 충분하다.
    2. 로봇이 멈출 때까지 대기 (is_fixed=True)
    3. 홈 복귀 명령 전송

    debounce 불필요: policy_gesture 는 전체 trajectory 를 commandBuffer 에 한 번에
    push 한 뒤 반환하므로 sub-move 사이 buffer 공백이 없다. is_fixed 가 거짓 → 참으로
    전환되는 시점이 곧 동작 완료 시점이다.
    """
    def _watcher():
        # 1단계: 움직임 시작 감지 (is_fixed=False)
        deadline_start = time.monotonic() + 1.5
        motion_started = False
        while time.monotonic() < deadline_start:
            if not get_state_fn().get("is_fixed", True):
                motion_started = True
                break
            time.sleep(0.05)

        if not motion_started:
            print("[Executor] 움직임 시작 미감지 → 홈 복귀 건너뜀")
            return

        # 2단계: 움직임 종료 감지 (is_fixed=True 첫 회)
        deadline_end = time.monotonic() + 20.0
        while time.monotonic() < deadline_end:
            if get_state_fn().get("is_fixed", True):
                break
            time.sleep(0.05)

        print("[Executor] 동작 완료 확인 → 홈 자세로 복귀")
        try:
            bot.send_command("h\n")
        except Exception as exc:
            print(f"⚠️ 홈 복귀 명령 전송 실패: {exc}")

    threading.Thread(target=_watcher, daemon=True).start()


def make_execute_node(executor: InterruptibleExecutor, bot, get_state_fn: Callable):
    """
    execute_node 함수를 생성해 반환한다.

    로봇 명령을 InterruptibleExecutor 로 비동기 실행한다.
    on_done 콜백은 execute_node 안에서 plan_type 을 포함해 직접 생성한다.

    홈 복귀 조건:
    - plan_type == "motion" 이고 cancelled=False 일 때만 수행한다.
    - 전송 완료 시점이 아니라 is_fixed=True(로봇 실제 정지) 확인 후 'h' 전송.
    """
    def execute_node(state: PhilState) -> PhilState:
        commands = state.get("commands", [])
        plan_type = state.get("plan_type", "none")

        if not commands:
            return state

        # on_done 콜백: plan_type 과 is_fixed 상태를 보고 홈 복귀 여부를 결정한다.
        def on_done(cancelled: bool):
            if cancelled:
                print("[Executor] 동작이 인터럽트로 중단됐습니다.")
                return
            if plan_type == "motion":
                _wait_for_fixed_then_home(bot, get_state_fn)

        executor.run(commands=commands, on_done=on_done)
        return state

    return execute_node


def make_return_home_node():
    """
    return_home_node — 그래프 흐름에서의 확장 포인트.
    실제 홈 복귀는 executor on_done 의 _wait_for_fixed_then_home 이 담당한다.
    이 노드는 향후 motion 완료 후 추가 처리(예: 로깅, 상태 갱신)를 붙일 자리다.
    """
    def return_home_node(state: PhilState) -> PhilState:
        print(f"[Graph] return_home_node: plan_type={state.get('plan_type')}")
        return state

    return return_home_node


# ------------------------------------------------------------------
# 전이 조건 (conditional edges)
# ------------------------------------------------------------------

def _decide_after_process(state: PhilState) -> str:
    """
    process_node 이후 어디로 갈지 결정한다.
    speech 또는 commands 가 있으면 execute 로 가서 TTS/실행을 처리한다.
    둘 다 없을 때만 end 로 빠진다.
    """
    if state.get("commands") or state.get("speech"):
        return "execute"
    return "end"


def _decide_after_execute(state: PhilState) -> str:
    """
    execute_node 이후 어디로 갈지 결정한다.
    motion 플랜이면 return_home 으로, 나머지는 end.
    """
    if state.get("plan_type") == "motion":
        return "return_home"
    return "end"


# ------------------------------------------------------------------
# 그래프 빌드
# ------------------------------------------------------------------

def build_phil_graph(
    bot,
    tts,
    stt_model,
    executor: InterruptibleExecutor,
    get_session: Callable,
    get_state_fn: Callable,
    run_brain_turn_fn: Callable,
    classifier_model: str,
    planner_model: str,
):
    """
    phil_robot LangGraph 그래프를 빌드하고 컴파일해 반환한다.

    phil_brain.py 가 startup 시에 한 번 호출한다.
    반환된 app 은 매 턴 app.invoke(state) 로 실행한다.

    get_state_fn: () -> dict  — 현재 로봇 상태 스냅샷을 반환한다.
                                is_fixed 폴링으로 홈 복귀 타이밍을 잡을 때 사용한다.
    """
    graph = StateGraph(PhilState)

    process_fn = make_process_node(
        bot=bot,
        tts=tts,
        stt_model=stt_model,
        executor=executor,
        get_session=get_session,
        run_brain_turn_fn=run_brain_turn_fn,
        classifier_model=classifier_model,
        planner_model=planner_model,
    )
    execute_fn = make_execute_node(executor=executor, bot=bot, get_state_fn=get_state_fn)
    return_home_fn = make_return_home_node()

    graph.add_node("process", process_fn)
    graph.add_node("execute", execute_fn)
    graph.add_node("return_home", return_home_fn)

    graph.set_entry_point("process")

    graph.add_conditional_edges(
        "process",
        _decide_after_process,
        {"execute": "execute", "end": END},
    )
    graph.add_conditional_edges(
        "execute",
        _decide_after_execute,
        {"return_home": "return_home", "end": END},
    )
    graph.add_edge("return_home", END)

    return graph.compile()
