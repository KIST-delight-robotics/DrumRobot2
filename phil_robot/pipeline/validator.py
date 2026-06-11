"""
plan 단위 validator 레이어.

planner 가 만든 결과를 바로 실행하지 않고,
skill 전개 -> 상대 동작 해석 -> 명령 검증 -> 메시지 보정까지 한 번에 책임진다.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .command_validator import (
    has_actionable_motion_command,
    validate_commands,
)
from .motion_resolver import resolve_motion_commands
from .play_modifier import PlayModifier, parse_play_modifier
from .skills import expand_skills
from .state_adapter import block_reason_of


@dataclass
class RepairHint:
    """
    validator 가 planner 명령을 거부했을 때, repair 도메인 planner 에게 돌려줄 사유.

    repair 루프가 이 hint 를 planner 입력에 실어, planner 가 명령을 다시 만들지 말고
    사용자에게 이유를 설명/되묻게 한다. failure_code 는 흐름 제어용 거친 분류이고,
    reason 은 사람이 읽을 수 있는 거부 이유(planner 가 풀어 설명할 재료)다.
    """

    failure_code: str = ""
    reason: str = ""
    rejected: List[str] = field(default_factory=list)


@dataclass
class ValidatedPlan:
    """
    planner 결과가 validator 를 지나면서 만들어지는 최종 실행 단위.

    이 객체부터는 executor 가 바로 소비할 수 있다.
    """

    skills: List[str] = field(default_factory=list)
    raw_op_cmds: List[str] = field(default_factory=list)
    expanded_op_cmds: List[str] = field(default_factory=list)
    resolved_op_cmds: List[str] = field(default_factory=list)
    valid_op_cmds: List[str] = field(default_factory=list)
    rejected_op_cmds: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    speech: str = ""
    reason: str = ""
    play_modifier: PlayModifier = field(default_factory=PlayModifier)
    # planner 명령이 거부돼 repair 가 필요할 때 채워진다.
    # None 이면 통과(또는 빈 명령). 채워져 있으면 graph 가 repair 루프로 보낸다.
    repair_hint: Optional[RepairHint] = None


def _content_failure_code(warnings: List[str]) -> str:
    """상태 차단이 아닌 명령 내용 거부 사유를 거친 코드로 정규화한다."""
    text = " ".join(warnings)
    if "지정되지 않음" in text:
        return "missing_info"
    if "한계" in text or "범위 초과" in text:
        return "joint_limit"
    if "곡 코드" in text:
        return "unknown_song"
    if "연주 명령 차단" in text:
        return "play_state"
    return "bad_command"


def build_play_modifier_message(play_modifier: PlayModifier) -> str:
    if play_modifier.tempo_scale < 1.0:
        return "연주 속도를 느리게 하겠습니다."
    if play_modifier.tempo_scale > 1.0:
        return "연주 속도를 빠르게 하겠습니다."
    if play_modifier.velocity_delta < 0:
        return "연주 세기를 약하게 하겠습니다."
    if play_modifier.velocity_delta > 0:
        return "연주 세기를 강하게 하겠습니다."
    return ""


def build_validated_plan(
    user_text: str,
    robot_state: Dict,
    classifier_output: Dict,
    planner_output: Dict,
) -> ValidatedPlan:
    """
    classifier + planner 결과를 실제 실행 가능한 plan 으로 정리한다.
    이 단계가 끝나면 phil_brain.py 는 더 이상 명령 보정 규칙을 알 필요가 없다.
    """
    skill_op_cmds, skill_warnings = expand_skills(planner_output.get("skills", []))
    planner_op_cmds = list(planner_output.get("op_cmd", planner_output.get("commands", [])))
    expanded_op_cmds = skill_op_cmds + planner_op_cmds

    resolution = resolve_motion_commands(user_text, expanded_op_cmds, robot_state)
    validation = validate_commands(resolution.op_cmds, robot_state)
    play_modifier = parse_play_modifier(user_text, robot_state)
    has_play_modifier = not play_modifier.is_identity()

    warnings = list(skill_warnings)
    warnings.extend(resolution.warnings)
    warnings.extend(validation.warnings)

    speech = planner_output.get("speech", "")

    # validator 는 speech 작가가 아니라 안전망이다.
    # play_modifier / 상대 동작 해석 같은 결정적 계산 결과만 speech 에 반영한다.
    # 막힘/범위/거부 안내는 planner 가 repair 도메인에서 직접 만든다(아래 repair_hint).
    if has_play_modifier:
        speech = build_play_modifier_message(play_modifier) or speech
    elif resolution.message_override:
        speech = resolution.message_override
    elif resolution.speech_override and has_actionable_motion_command(validation.valid_commands):
        speech = resolution.speech_override

    # planner 명령이 (전부) 거부돼 실행할 동작이 없으면 repair 사유를 만든다.
    # graph 가 이 hint 를 repair 도메인 planner 로 돌려보내 설명/되묻기를 생성하게 한다.
    repair_hint = None
    if validation.rejected_commands and not has_actionable_motion_command(validation.valid_commands):
        block = block_reason_of(robot_state)
        failure_code = block if block != "none" else _content_failure_code(validation.warnings)
        repair_hint = RepairHint(
            failure_code=failure_code,
            reason="; ".join(w for w in validation.warnings if w),
            rejected=list(validation.rejected_commands),
        )

    return ValidatedPlan(
        skills=list(planner_output.get("skills", [])),
        raw_op_cmds=planner_op_cmds,
        expanded_op_cmds=expanded_op_cmds,
        resolved_op_cmds=list(resolution.op_cmds),
        valid_op_cmds=list(validation.valid_commands),
        rejected_op_cmds=list(validation.rejected_commands),
        warnings=warnings,
        speech=speech,
        reason=planner_output.get("reason", ""),
        play_modifier=play_modifier,
        repair_hint=repair_hint,
    )
