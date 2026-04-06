"""
plan 단위 validator 레이어.

planner 가 만든 결과를 바로 실행하지 않고,
skill 전개 -> 상대 동작 해석 -> 명령 검증 -> 메시지 보정까지 한 번에 책임진다.
"""

from dataclasses import dataclass, field
from typing import Dict, List

try:
    from .command_validator import (
        ValidationResult,
        has_actionable_motion_command,
        user_text_requests_motion,
        validate_commands,
    )
    from .failure import build_motion_block_message
    from .motion_resolver import resolve_motion_commands
    from .play_modifier import PlayModifier, parse_play_modifier
    from .skills import expand_skills

except ImportError:
    from command_validator import (
        ValidationResult,
        has_actionable_motion_command,
        user_text_requests_motion,
        validate_commands,
    )
    from failure import build_motion_block_message
    from motion_resolver import resolve_motion_commands
    from play_modifier import PlayModifier, parse_play_modifier
    from skills import expand_skills


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


def build_partial_execution_message(valid_op_cmds: List[str], rejected_op_cmds: List[str]) -> str:
    """
    일부 명령만 실행 가능한 경우 사용자에게 그 사실을 명확히 알린다.

    너무 세부적인 reject 이유를 장황하게 나열하기보다,
    "가능한 동작은 수행하고 일부는 제한으로 제외했다"는 사실을 우선 전달한다.
    """
    if not valid_op_cmds or not rejected_op_cmds:
        return ""

    return "가능한 동작만 먼저 수행할게요. 일부 동작은 범위나 현재 상태 제한 때문에 제외했습니다."


def build_validated_plan(
    user_text: str,
    robot_state: Dict,
    classifier_result: Dict,
    planner_result: Dict,
) -> ValidatedPlan:
    """
    classifier + planner 결과를 실제 실행 가능한 plan 으로 정리한다.
    이 단계가 끝나면 phil_brain.py 는 더 이상 명령 보정 규칙을 알 필요가 없다.
    """
    skill_op_cmds, skill_warnings = expand_skills(planner_result.get("skills", []))
    planner_op_cmds = list(planner_result.get("op_cmd", planner_result.get("commands", [])))
    expanded_op_cmds = skill_op_cmds + planner_op_cmds

    resolution = resolve_motion_commands(user_text, expanded_op_cmds, robot_state)
    validation = validate_commands(resolution.op_cmds, robot_state)
    parsed_modifier = parse_play_modifier(user_text)
    has_play_intent = classifier_result.get("intent") == "play_request"
    has_valid_play = any(command.startswith("p:") for command in validation.valid_commands)
    if has_play_intent and has_valid_play and not parsed_modifier.is_identity():
        play_modifier = parsed_modifier
    else:
        play_modifier = None

    warnings = list(skill_warnings)
    warnings.extend(resolution.warnings)
    warnings.extend(validation.warnings)

    speech = planner_result.get("speech", "")

    # planner 가 거절 대사를 만들지 못해도 validator 가 최종 사용자 메시지를 보수적으로 보정한다.
    if resolution.message_override:
        speech = resolution.message_override
    elif resolution.speech_override and has_actionable_motion_command(validation.valid_commands):
        speech = resolution.speech_override
    elif has_actionable_motion_command(validation.valid_commands) and validation.rejected_commands:
        speech = build_partial_execution_message(validation.valid_commands, validation.rejected_commands)
    elif classifier_result.get("needs_motion", False) and not has_actionable_motion_command(validation.valid_commands):
        speech = build_motion_block_message(robot_state)
    elif (
        classifier_result.get("intent") == "motion_request"
        and user_text_requests_motion(user_text)
        and not has_actionable_motion_command(validation.valid_commands)
    ):
        speech = build_motion_block_message(robot_state)

    return ValidatedPlan(
        skills=list(planner_result.get("skills", [])),
        raw_op_cmds=planner_op_cmds,
        expanded_op_cmds=expanded_op_cmds,
        resolved_op_cmds=list(resolution.op_cmds),
        valid_op_cmds=list(validation.valid_commands),
        rejected_op_cmds=list(validation.rejected_commands),
        warnings=warnings,
        speech=speech,
        reason=planner_result.get("reason", ""),
        play_modifier=play_modifier,
    )
