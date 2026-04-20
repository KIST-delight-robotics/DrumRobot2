# phil_robot/pipeline/session.py
"""
한 대화 세션 동안 유지되는 단기 기억(short-term memory) 계층.

phil_brain.py의 메인 루프가 SessionContext를 소유하고,
매 턴이 끝날 때 update_session()으로 갱신한다.
LLM 호출 없이 deterministic하게만 상태를 업데이트한다.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

# planner input에 노출할 최근 턴 수 (token 예산 고려)
MAX_HISTORY_TURNS = 5
PLANNER_VISIBLE_TURNS = 3


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
    3) pending_*     : clarification 대기 상태
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

    # ── clarification 대기 상태 ───────────────────────────────────────────
    # pending_clarification_q가 채워져 있으면,
    # 다음 user_text는 이 질문에 대한 답변으로 처리한다.
    pending_user_text: Optional[str] = None        # 원래 사용자 발화 ("허리 돌려")
    pending_clarification_q: Optional[str] = None  # 필이 되물은 질문 ("몇 도로?")

    # TODO: 향후 확장 포인트
    # pending_task: Optional[Dict] = None
    #   → "연주 끝나면 인사해" 같은 조건부 작업 대기열
    # user_name: Optional[str] = None
    #   → 사용자 이름 기억 ("제 이름은 민수예요" → 이후 "안녕하세요 민수님")
    # user_preferences: Dict = field(default_factory=dict)
    #   → 세션 내 선호도 ("빠른 연주 좋아함", "조용히 말해줘" 등)


def resolve_clarification_text(ctx: SessionContext, user_text: str) -> str:
    """
    clarification 대기 상태라면 원래 발화와 이번 답변을 합친다.

    예) pending_user_text="허리 돌려", user_text="30도"
        → "허리 돌려 30도"
    합친 텍스트는 classifier/planner에 그대로 전달된다.
    """
    if ctx.pending_clarification_q and ctx.pending_user_text:
        return f"{ctx.pending_user_text} {user_text}"
    return user_text


def update_session(
    ctx: SessionContext,
    user_text: str,
    brain_result,
) -> SessionContext:
    """
    한 턴이 끝난 후 SessionContext를 갱신한다.
    brain_result는 BrainTurnResult 타입이다.

    호출 시점: phil_brain.py에서 execute/speak 직후.
    """
    validated = brain_result.validated_plan

    # ── 대화 히스토리 추가 ────────────────────────────────────────────────
    ctx.history.append(TurnRecord(user_text=user_text, phil_speech=validated.speech))
    if len(ctx.history) > MAX_HISTORY_TURNS:
        ctx.history = ctx.history[-MAX_HISTORY_TURNS:]

    # ── 마지막 상태 갱신 ──────────────────────────────────────────────────
    ctx.last_intent = brain_result.classifier_result.get("intent", "")
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

    # ── clarification 상태 처리 ───────────────────────────────────────────
    # 기존 pending을 먼저 초기화한다.
    ctx.pending_user_text = None
    ctx.pending_clarification_q = None

    # 이번 턴 planner가 clarification을 요청했으면 pending 상태로 전환한다.
    if validated.clarification_question:
        ctx.pending_user_text = user_text
        ctx.pending_clarification_q = validated.clarification_question

    return ctx


def build_session_summary(ctx: SessionContext) -> Optional[Dict]:
    """
    planner input JSON에 포함할 session 요약을 만든다.
    비어있는 세션이면 None을 반환해 planner input을 깔끔하게 유지한다.
    """
    if not ctx.history and not ctx.last_joint and not ctx.last_look and not ctx.last_play:
        return None

    summary: Dict = {}

    # 최근 대화 (PLANNER_VISIBLE_TURNS 턴만 노출해 token을 아낀다)
    if ctx.history:
        summary["recent_turns"] = [
            {"user": t.user_text, "phil": t.phil_speech}
            for t in ctx.history[-PLANNER_VISIBLE_TURNS:]
        ]

    # 마지막 동작 상태 (상대 이동 지시 해석에 사용)
    if ctx.last_joint and ctx.last_angle is not None:
        summary["last_joint"] = ctx.last_joint
        summary["last_angle"] = ctx.last_angle
    if ctx.last_look:
        summary["last_look"] = ctx.last_look
    if ctx.last_play:
        summary["last_play"] = ctx.last_play

    return summary if summary else None
