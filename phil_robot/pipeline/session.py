# phil_robot/pipeline/session.py
"""
한 대화 세션 동안 유지되는 단기 기억(short-term memory) 계층.

phil_brain.py의 메인 루프가 SessionContext를 소유하고,
매 턴이 끝날 때 update_session()으로 갱신한다.
LLM 호출 없이 deterministic하게만 상태를 업데이트한다.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .command_validator import has_actionable_motion_command

# planner input에 노출할 최근 턴 수 (token 예산 고려)
MAX_HISTORY_TURNS = 5
PLANNER_VISIBLE_TURNS = 3

# 이 intent 들은 "동작을 해야 하는" 요청이다. 이런 턴이 실행 명령 없이 끝나면
# (되묻기/차단/fallback) cross-turn recovery 대상이 된다.
_ACTIONABLE_INTENTS = {"motion_request", "play_request"}


def _needs_action(classifier_output: Dict) -> bool:
    intent = classifier_output.get("intent", "")
    return intent in _ACTIONABLE_INTENTS or bool(classifier_output.get("needs_motion"))


@dataclass
class TurnRecord:
    """한 턴의 대화 기록: 사용자 발화 + 필의 응답."""
    user_text: str
    phil_speech: str


@dataclass
class SessionContext:
    """
    한 대화 세션의 단기 기억 전체.

    크게 세 구역으로 나뉜다.
    1) history       : 최근 N턴의 대화 기록
    2) last_*        : 마지막으로 확인된 로봇 동작 상태
    3) recovery_*    : cross-turn 복구(되묻기 이어가기) 대기 상태
    """

    # ── 대화 히스토리 ──────────────────────────────────────────────────────
    history: List[TurnRecord] = field(default_factory=list)

    # ── 마지막으로 확인된 동작 상태 ────────────────────────────────────────
    # "거기서 더 올려", "아까처럼" 같은 지시를 처리할 때 참조한다.
    last_intent: str = ""
    last_joint: Optional[str] = None    # 마지막으로 움직인 관절 (예: "L_wrist")
    last_angle: Optional[float] = None  # 마지막 관절 각도 (도)
    last_look: Optional[str] = None     # 마지막 look 명령 (예: "look:30,90")
    last_play: Optional[str] = None     # 마지막 play 곡 코드 (예: "TIM")
    last_speech: str = ""               # 필의 마지막 발화

    # ── cross-turn recovery 대기 상태 ─────────────────────────────────────
    # 동작 요청이 한 턴에 해결되지 않으면(되묻기/차단) 원래 요청을 pending 으로 들고
    # 다음 턴에 이어간다. recovery_count 가 한도(robot_fsm.MAX_RECOVERY)에 닿으면 giveup.
    recovery_count: int = 0                    # 이 복구 스레드가 미해결로 넘어간 턴 수
    pending_intent: Optional[str] = None       # 원래 actionable 발화 ("허리 돌려")
    pending_classifier: Optional[Dict] = None  # 원래 classifier 결과 (continuation 시 재사용)

    # TODO: 향후 확장 포인트
    # pending_task: Optional[Dict] = None
    #   → "연주 끝나면 인사해" 같은 조건부 작업 대기열
    # user_name: Optional[str] = None
    #   → 사용자 이름 기억 ("제 이름은 민수예요" → 이후 "안녕하세요 민수님")
    # user_preferences: Dict = field(default_factory=dict)
    #   → 세션 내 선호도 ("빠른 연주 좋아함", "조용히 말해줘" 등)


def update_session(
    ctx: SessionContext,
    user_text: str,
    classifier_output: Dict,
    validated,
) -> SessionContext:
    """
    한 턴이 끝난 후 SessionContext를 갱신한다.
    classifier_output: dict, validated: ValidatedPlan.
    (이전에는 BrainTurnResult 를 받았으나, 런타임이 PhilState 만 다루도록
     필요한 조각만 직접 받는다.)

    호출 시점: phil_brain.py에서 execute/speak 직후.
    """
    # ── 대화 히스토리 추가 ────────────────────────────────────────────────
    ctx.history.append(TurnRecord(user_text=user_text, phil_speech=validated.speech))
    if len(ctx.history) > MAX_HISTORY_TURNS:
        ctx.history = ctx.history[-MAX_HISTORY_TURNS:]

    # ── 마지막 상태 갱신 ──────────────────────────────────────────────────
    ctx.last_intent = classifier_output.get("intent", "")
    ctx.last_speech = validated.speech

    # 실행된 명령에서 관절/시선/연주 상태를 추출한다.
    for cmd in validated.valid_op_cmds:
        if cmd.startswith("move:"):
            # "move:L_wrist,75.0" 형식 파싱
            try:
                _, move_args = cmd.split(":", 1)
                joint, angle_raw = move_args.split(",", 1)
                ctx.last_joint = joint
                ctx.last_angle = float(angle_raw)
            except ValueError:
                pass
        elif cmd.startswith("look:"):
            ctx.last_look = cmd
        elif cmd.startswith("p:"):
            # "p:TIM" → "TIM"
            ctx.last_play = cmd.split(":", 1)[1]

    # ── cross-turn recovery 갱신 ──────────────────────────────────────────
    # 동작이 필요한 턴(actionable)인데 실행할 명령이 안 나왔으면(되묻기/차단/fallback)
    # 미해결로 보고 recovery 를 진행한다. 그 외(실행 성공/일반 chat·status)는 스레드 종료.
    unresolved = _needs_action(classifier_output) and not has_actionable_motion_command(
        validated.valid_op_cmds
    )
    if unresolved:
        ctx.recovery_count += 1
        if ctx.pending_intent is None:   # 첫 미해결 턴의 원래 요청을 고정해 둔다.
            ctx.pending_intent = user_text
            ctx.pending_classifier = dict(classifier_output)
    else:
        ctx.recovery_count = 0
        ctx.pending_intent = None
        ctx.pending_classifier = None

    return ctx


def build_session_summary(ctx: SessionContext) -> Optional[Dict]:
    """
    planner input JSON에 포함할 session 요약을 만든다.
    비어있는 세션이면 None을 반환해 planner input을 깔끔하게 유지한다.
    """
    if (
        not ctx.history
        and not ctx.last_joint
        and not ctx.last_look
        and not ctx.last_play
        and not ctx.pending_intent
    ):
        return None

    summary: Dict = {}

    # 최근 대화 (PLANNER_VISIBLE_TURNS 턴만 노출해 token을 아낀다)
    if ctx.history:
        summary["recent_turns"] = [
            {"user": t.user_text, "phil": t.phil_speech}
            for t in ctx.history[-PLANNER_VISIBLE_TURNS:]
        ]

    # cross-turn 복구 중이면 원래 요청을 함께 넘겨 planner 가 이번 발화와 합쳐 해석하게 한다.
    # (예: pending_intent="허리 돌려" + user_text="30도" → 허리를 30도)
    if ctx.pending_intent:
        summary["pending_intent"] = ctx.pending_intent

    # 마지막 동작 상태 (상대 이동 지시 해석에 사용)
    if ctx.last_joint and ctx.last_angle is not None:
        summary["last_joint"] = ctx.last_joint
        summary["last_angle"] = ctx.last_angle
    if ctx.last_look:
        summary["last_look"] = ctx.last_look
    if ctx.last_play:
        summary["last_play"] = ctx.last_play

    return summary if summary else None
