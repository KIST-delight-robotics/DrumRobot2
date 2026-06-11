"""
phil_robot per-turn FSM (imperative).

한 사용자 턴의 흐름을 step 들의 고정 순서 + repair 루프 하나로 표현한다.
예전엔 경량 StateGraph(langgraph shim)로 노드/엣지를 선언했지만, 흐름이
"고정 체인 + 분기 1~2개" 수준이라 graph 추상화의 이득(선언적 edge, langgraph 이관)이
없어 imperative 로 바꿨다. (Jetson Python 3.8 에서는 진짜 langgraph 설치/이관도 사실상 불가.)
개념상 state(step)와 transition 이 있는 작은 상태기계지만, 명시적 전이표 대신
run_turn() 의 호출 순서 + for 루프로 엮는다.

  preprocess → classify → state → direct_answer → (planner ⇄ validator) → execute

단계별 책임:
  preprocess     : prefilter(pause/resume/인사). 맞으면 _shortcut 으로 표시.
  classify       : classifier LLM 으로 intent 결정 (robot_state 불필요). shortcut 이면 통과.
  state          : planner 직전 fresh robot_state fetch (cross-turn 복구의 핵심).
  direct_answer  : 상태/정체/레퍼토리 직답 shortcut. 맞으면 planner 통과.
  planner⇄validator : repair 루프. validator 가 거부하면 사유(repair_hint)를 실어
                      planner(repair 도메인)로 재호출. 최대 MAX_REPAIR 회, 소진 시 fallback.
  execute        : Executor 로 명령 비동기 전송. plan_type==motion 이면 Home Watcher.

PhilState 딕셔너리 하나가 단계 사이를 굴러다니고, 각 step 은 필요한 키만 갱신한다.
prefilter/direct_answer 가 답을 만들면 `_shortcut` 으로 LLM step(classify/planner)을 건너뛴다.
단 경로 자체는 그대로라 validator(skill 전개·최종 commands/speech 추출)와 execute 는 거친다
— executor 직행이 아니다. (rule-base 결과도 validator 를 통해 최종 명령/발화로 변환된다.)

cross-turn recovery 는 세 군데로 나뉜다: 지속 상태(recovery_count/pending_intent)는 session,
턴-간 반복은 phil_brain 의 while 루프, 매 턴 진입에서 그 session 상태를 읽어 giveup/continue 를
고르는 분기는 run_turn(여기) 이 한다.
"""

import threading
import time
from typing import TYPE_CHECKING, Callable, Dict, List, TypedDict

from .brain_pipeline import (
    build_direct_answer_plan,
    build_prefilter_plan,
    classify_step,
    planner_step,
)
from .command_validator import has_actionable_motion_command
from .exec_thread import Executor
from .failure import FALLBACK_MESSAGE
from .planner import PLANNER_DOMAIN_REPAIR, select_planner_domain
from .skills import get_skill_categories
from .state_adapter import adapt_robot_state
from .validator import build_validated_plan

if TYPE_CHECKING:
    from .validator import ValidatedPlan

# 한 턴 안에서 planner⇄validator 가 핑퐁할 수 있는 최대 repair 호출 수.
# 보통 1회면 수렴한다(거부 → planner 가 빈 명령+설명). 소진되면 fallback 으로 끝낸다.
MAX_REPAIR = 2

# cross-turn 복구가 이 횟수만큼 미해결로 이어지면 planner 없이 giveup(결정적 리셋)한다.
# (사람이 MAX_RECOVERY 번 시도하도록 두고, 그 다음 턴은 deterministic 안내로 끝낸다.)
MAX_RECOVERY = 4
_GIVEUP_MESSAGE = "죄송해요, 잘 이해하지 못했어요. 처음부터 다시 말씀해 주세요."
_CANCEL_WORDS = {"취소", "취소해", "아니", "아니야", "됐어", "관둬", "안해", "안 해"}

# 이 intent 들은 "동작을 해야 하는" 요청이다. 빈 계획으로 끝나면 되묻기(repair/recovery) 대상.
_ACTIONABLE_INTENTS = {"motion_request", "play_request"}


def _needs_action(classifier_output: dict) -> bool:
    intent = classifier_output.get("intent", "")
    return intent in _ACTIONABLE_INTENTS or bool(classifier_output.get("needs_motion"))


class PhilState(TypedDict, total=False):
    """
    한 턴의 step 사이를 전달되는 데이터 묶음.
    run_turn() 이 user_text 로 초기화하고, 각 step 이 필요한 키만 갱신한다.
    """
    # ── 입력 ──────────────────────────────────────────────────────────
    user_text: str

    # ── 흐름 제어 ─────────────────────────────────────────────────────
    _shortcut: bool        # prefilter/direct_answer 가 planner_output 을 이미 만들었는가
    _recovery: bool        # cross-turn 복구 continuation (classify 건너뛰고 pending 도메인 이어감)
    repair_attempt: int    # 이번 턴 repair 도메인 재호출 횟수 (디버그용; 루프는 run_turn 이 제어)
    repair_hint: dict      # validator 가 거부하며 돌려준 사유. 비어있지 않으면 planner 가 repair 호출

    # ── classify ──────────────────────────────────────────────────────
    classifier_output: dict
    planner_domain: str

    # ── state ─────────────────────────────────────────────────────────
    robot_state: dict

    # ── planner ───────────────────────────────────────────────────────
    planner_output: dict

    # ── validator (최종 출력) ─────────────────────────────────────────
    plan_type: str        # "motion" | "play" | "stop" | "chat" | "none"
    speech: str           # 필이 할 말
    commands: List[str]   # 실제 전송할 명령 목록
    play_modifier: Dict   # tempo_scale / velocity_delta / source / apply_scope (play 전용)
    validated: object     # ValidatedPlan (session 갱신/디버그용)

    # ── 디버그 (런타임 출력용, eval 과 무관) ──────────────────────────
    debug: dict


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
    has_stop = any(c == "pause" for c in cmds)
    has_move = any(c.startswith("move:") or c.startswith("gesture:") for c in cmds)
    has_look_only = all(c.startswith(_LOOK_ONLY_PREFIXES) for c in cmds)

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


def _play_modifier_to_dict(validated_plan: "ValidatedPlan") -> Dict:
    modifier = getattr(validated_plan, "play_modifier", None)
    if modifier is None:
        return {}
    return {
        "tempo_scale": getattr(modifier, "tempo_scale", 1.0),
        "velocity_delta": getattr(modifier, "velocity_delta", 0),
        "source": getattr(modifier, "source", None),
        "apply_scope": getattr(modifier, "apply_scope", None),
    }


# ------------------------------------------------------------------
# Step 빌더 — 클로저로 외부 의존성(session getter, 모델명 등)을 주입한다.
# ------------------------------------------------------------------

def make_preprocess_step():
    """
    preprocess: prefilter(pause/resume/인사).
    맞으면 classifier 호출 없이 planner_output 을 채우고 `_shortcut=True` 로 표시한다.
    (이전 실행 인터럽트는 wait 제거로 불필요해 삭제. clarification 합치기는 폐기 —
     cross-turn 이어가기는 phil_brain + session 의 recovery 가 담당한다.)
    """
    def preprocess(state: PhilState) -> PhilState:
        prefilter = build_prefilter_plan(state["user_text"])
        if prefilter is None:
            return state

        classifier_output, planner_output, planner_domain = prefilter
        return {
            **state,
            "classifier_output": classifier_output,
            "planner_output": planner_output,
            "planner_domain": planner_domain,
            "_shortcut": True,
        }

    return preprocess


def make_classify_step(classifier_model: str):
    """classify: classifier LLM 호출. shortcut/recovery continuation 이면 통과."""
    def classify(state: PhilState) -> PhilState:
        if state.get("_shortcut") or state.get("_recovery"):
            return state

        classifier_output, diag = classify_step(
            state["user_text"], classifier_model, capture_metrics=True
        )
        planner_domain = select_planner_domain(classifier_output)

        debug = dict(state.get("debug", {}))
        debug["classifier_input"] = diag["classifier_input"]
        debug["classifier_duration_sec"] = diag["duration_sec"]
        debug["classifier_metrics"] = diag["metrics"]

        return {
            **state,
            "classifier_output": classifier_output,
            "planner_domain": planner_domain,
            "debug": debug,
        }

    return classify


def make_state_step(get_state_fn: Callable):
    """
    state: planner 직전에 fresh robot_state 를 fetch 한다.
    classifier LLM latency 만큼 지난 뒤의 최신 상태를 planner/validator 가 보게 하고,
    cross-turn 복구(예: '키 뽑았어' 다음 턴)에서 세상 변화를 반영하는 핵심 지점이다.
    """
    def fetch_state(state: PhilState) -> PhilState:
        robot_state = adapt_robot_state(get_state_fn())
        return {**state, "robot_state": robot_state}

    return fetch_state


def make_direct_answer_step():
    """direct_answer: 상태/정체/레퍼토리 직답 shortcut. shortcut/recovery 면 건너뜀."""
    def direct_answer(state: PhilState) -> PhilState:
        if state.get("_shortcut") or state.get("_recovery"):
            return state

        direct_plan = build_direct_answer_plan(
            state["user_text"], state["classifier_output"], state["robot_state"]
        )
        if direct_plan is not None:
            return {**state, "planner_output": direct_plan, "_shortcut": True}
        return state

    return direct_answer


def make_planner_step(planner_model: str, get_session: Callable):
    """
    planner: planner LLM 호출.

    - repair_hint 가 있으면(직전 validator 거부) repair 도메인으로 재호출한다.
      repair 는 shortcut 보다 우선한다 — 거부된 shortcut(예: 막힘 상태의 인사 제스처)도
      repair 로 설명/되묻기를 만들어야 하기 때문이다.
    - repair_hint 도 없고 shortcut 이면 통과.
    - 그 외에는 intent 도메인으로 일반 계획.
    """
    def planner(state: PhilState) -> PhilState:
        repair_hint = state.get("repair_hint") or {}

        if repair_hint:
            domain = PLANNER_DOMAIN_REPAIR
            repair_attempt = state.get("repair_attempt", 0) + 1
        elif state.get("_shortcut"):
            return state
        else:
            domain = state["planner_domain"]
            repair_attempt = state.get("repair_attempt", 0)

        session = get_session()
        planner_output, diag = planner_step(
            state["robot_state"],
            state["user_text"],
            state["classifier_output"],
            domain,
            session,
            planner_model,
            capture_metrics=True,
            repair_hint=(repair_hint or None),
        )

        debug = dict(state.get("debug", {}))
        debug["planner_input"] = diag["planner_input"]
        # repair 로 여러 번 부르면 planner 시간을 누적해 둔다.
        debug["planner_duration_sec"] = debug.get("planner_duration_sec", 0.0) + diag["duration_sec"]
        debug["planner_metrics"] = diag["metrics"]

        return {
            **state,
            "planner_output": planner_output,
            "planner_domain": domain,
            "repair_attempt": repair_attempt,
            "repair_hint": {},   # 소비됨. validator 가 또 거부하면 새로 채운다.
            "debug": debug,
        }

    return planner


def make_validator_step():
    """validator: build_validated_plan 으로 speech/commands/plan_type 확정 + repair_hint 추출."""
    def validator(state: PhilState) -> PhilState:
        validated = build_validated_plan(
            user_text=state["user_text"],
            robot_state=state["robot_state"],
            classifier_output=state["classifier_output"],
            planner_output=state["planner_output"],
        )
        classifier_intent = state["classifier_output"].get("intent", "none")
        plan_type = _infer_plan_type(validated, classifier_intent)
        commands = _extract_commands(validated)

        # 거부 사유가 있으면 repair 루프로 보낼 hint 를 PhilState 에 싣는다.
        hint = getattr(validated, "repair_hint", None)
        repair_hint: Dict = {}
        if hint is not None:
            repair_hint = {
                "failure_code": hint.failure_code,
                "reason": hint.reason,
                "rejected": list(hint.rejected),
            }

        # missing-info: 첫 시도(repair 도메인이 아님)인데 동작이 필요한 의도가 실행 명령을
        # 하나도 못 냈으면(빈 계획) repair 로 보내 "무엇을/몇 도?"를 되묻게 한다.
        # repair 도메인 출력은 빈 계획이 정상이므로 이 트리거에서 제외한다(무한 루프 방지).
        if (
            not repair_hint
            and state.get("planner_domain") != PLANNER_DOMAIN_REPAIR
            and _needs_action(state["classifier_output"])
            and not has_actionable_motion_command(getattr(validated, "valid_op_cmds", []))
        ):
            repair_hint = {
                "failure_code": "missing_info",
                "reason": "요청한 동작에 필요한 정보(목표 각도/대상/곡)가 부족합니다.",
                "rejected": [],
            }

        return {
            **state,
            "validated": validated,
            "speech": validated.speech or "",
            "commands": commands,
            "plan_type": plan_type,
            "play_modifier": _play_modifier_to_dict(validated),
            "repair_hint": repair_hint,
        }

    return validator


def make_fallback_step():
    """
    fallback: repair 루프가 끝내 수렴 못 한 희귀 케이스.
    명령을 전부 버리고(안전) 결정적 안전 문구만 남긴다.
    """
    def fallback(state: PhilState) -> PhilState:
        return {
            **state,
            "speech": FALLBACK_MESSAGE,
            "commands": [],
            "plan_type": "none",
            "repair_hint": {},
        }

    return fallback


def home(bot, get_state_fn: Callable) -> None:
    """
    동작 완료(is_fixed=True) 감지 후 홈 복귀 'h' 를 보내는 데몬 스레드를 띄우고 즉시 반환한다.
    스레드 본체 _watch 를 안에 두어 "스레드 생성 + 움직임 감지"를 한 함수로 묶는다.
    fire-and-forget 데몬이라 Thread 객체는 따로 보관하지 않는다(join/cancel 불필요).
    """
    def _watch():
        # 1. 움직임 시작 대기 (is_fixed=False, 최대 1.5초). 미감지면 홈 복귀 건너뜀.
        deadline = time.monotonic() + 1.5
        started = False
        while time.monotonic() < deadline:
            if not get_state_fn().get("is_fixed", True):
                started = True
                break
            time.sleep(0.05)
        if not started:
            print("[Executor] 움직임 시작 미감지 → 홈 복귀 건너뜀")
            return

        # 2. 정지 대기 (is_fixed=True 첫 회, 최대 20초)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if get_state_fn().get("is_fixed", True):
                break
            time.sleep(0.05)

        # 3. 홈 복귀 전송
        print("[Executor] 동작 완료 확인 → 홈 자세로 복귀")
        try:
            bot.send_command("h\n")
        except Exception as exc:
            print(f"⚠️ 홈 복귀 명령 전송 실패: {exc}")

    threading.Thread(target=_watch, daemon=True).start()


def make_execute_step(executor: Executor, bot, get_state_fn: Callable):
    """
    execute: 로봇 명령을 Executor 로 비동기 전송한다.
    on_done 콜백은 plan_type 이 motion 일 때 Home Watcher 를 띄운다.
    (전송 완료 시점이 아니라 is_fixed=True 확인 후 'h' 전송)
    """
    def execute(state: PhilState) -> PhilState:
        commands = state.get("commands", [])
        plan_type = state.get("plan_type", "none")

        if not commands:
            return state

        def on_done():
            if plan_type == "motion":
                home(bot, get_state_fn)

        executor.execute(commands=commands, on_done=on_done)
        return state

    return execute


# ------------------------------------------------------------------
# run_turn 빌드
# ------------------------------------------------------------------

def build_run_turn(
    bot,
    executor: Executor,
    get_session: Callable,
    get_state_fn: Callable,
    classifier_model: str,
    planner_model: str,
):
    """
    한 턴을 처리하는 run_turn(user_text) 함수를 만들어 반환한다.
    phil_brain.py 가 startup 에 한 번 호출하고, 매 턴 run_turn(user_text) 로 실행한다.

    get_state_fn: () -> dict — 현재 로봇 상태 스냅샷. state step 의 fresh fetch 와
                               홈 복귀 타이밍 폴링에 쓴다.
    """
    preprocess = make_preprocess_step()
    classify = make_classify_step(classifier_model)
    fetch_state = make_state_step(get_state_fn)
    direct_answer = make_direct_answer_step()
    planner = make_planner_step(planner_model, get_session)
    validator = make_validator_step()
    fallback = make_fallback_step()
    execute = make_execute_step(executor, bot, get_state_fn)

    def run_turn(user_text: str) -> PhilState:
        # cross-turn 복구 상태를 turn 진입에서 읽는다 (지속 상태는 session 이 보관).
        session = get_session()
        recovery_count = session.recovery_count if session is not None else 0
        pending_intent = session.pending_intent if session is not None else None
        pending_classifier = session.pending_classifier if session is not None else None

        state: PhilState = {
            "user_text": user_text,
            "_shortcut": False,
            "_recovery": False,
            "repair_attempt": 0,
            "repair_hint": {},
            "classifier_output": {},
            "planner_domain": "",
            "planner_output": {},
            "debug": {},
        }

        if pending_intent and recovery_count >= MAX_RECOVERY:
            # 복구가 한도를 넘겼다 → planner 없이 결정적 리셋 안내.
            # _shortcut 으로 classify/planner 를 건너뛰고 validator 가 빈 계획을 확정한다.
            # (이 턴은 chat 으로 분류되어 update_session 이 복구 스레드를 리셋한다.)
            state["classifier_output"] = {"intent": "chat"}
            state["planner_output"] = {"skills": [], "op_cmd": [], "speech": _GIVEUP_MESSAGE, "reason": "recovery 한도 초과"}
            state["_shortcut"] = True
        else:
            state = preprocess(state)   # prefilter (pause/resume/인사)
            if not state["_shortcut"] and pending_intent:
                # 복구 진행 중이고 prefilter 도 아니다 → 원래 요청을 이어간다.
                if user_text.strip() in _CANCEL_WORDS:
                    state["classifier_output"] = {"intent": "chat"}
                    state["planner_output"] = {"skills": [], "op_cmd": [], "speech": "알겠습니다. 그 요청은 취소할게요.", "reason": "사용자 취소"}
                    state["_shortcut"] = True
                else:
                    # classify 를 건너뛰고 원래 classifier/도메인을 재사용한다.
                    # pending_intent 는 session_summary 로 planner 에 전달돼 이번 발화와 합쳐진다.
                    state["classifier_output"] = dict(pending_classifier or {})
                    state["planner_domain"] = select_planner_domain(pending_classifier or {})
                    state["_recovery"] = True

        state = classify(state)        # shortcut/recovery 면 통과
        state = fetch_state(state)
        state = direct_answer(state)   # shortcut/recovery 면 통과

        # repair 루프: planner → validator, 거부되면 사유 싣고 다시 planner.
        # MAX_REPAIR 회 안에 수렴 못 하면(else) fallback.
        for _ in range(MAX_REPAIR + 1):
            state = planner(state)
            state = validator(state)
            if not state["repair_hint"]:
                break
        else:
            state = fallback(state)

        state = execute(state)
        return state

    return run_turn
