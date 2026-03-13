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
        build_motion_block_message,
        has_actionable_motion_command,
        user_text_requests_motion,
        validate_commands,
    )
    from .motion_resolver import resolve_motion_commands
    from .skills import expand_skills
except ImportError:
    from command_validator import (
        ValidationResult,
        build_motion_block_message,
        has_actionable_motion_command,
        user_text_requests_motion,
        validate_commands,
    )
    from motion_resolver import resolve_motion_commands
    from skills import expand_skills


@dataclass
class ValidatedPlan:
    """
    planner 결과가 validator 를 지나면서 만들어지는 최종 실행 단위.

    이 객체부터는 executor 가 바로 소비할 수 있다.
    """

    skills: List[str] = field(default_factory=list)
    raw_commands: List[str] = field(default_factory=list)
    expanded_commands: List[str] = field(default_factory=list)
    resolved_commands: List[str] = field(default_factory=list)
    valid_commands: List[str] = field(default_factory=list)
    rejected_commands: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    speech: str = ""
    reason: str = ""


def build_validated_plan(user_text: str, robot_state: Dict, classifier_result: Dict, planner_result: Dict) -> ValidatedPlan:
    """
    classifier + planner 결과를 실제 실행 가능한 plan 으로 정리한다.
    이 단계가 끝나면 phil_brain.py 는 더 이상 명령 보정 규칙을 알 필요가 없다.
    """
    skill_commands, skill_warnings = expand_skills(planner_result.get("skills", []))
    expanded_commands = skill_commands + list(planner_result.get("commands", []))

    resolution = resolve_motion_commands(user_text, expanded_commands, robot_state)
    validation = validate_commands(resolution.commands, robot_state)

    warnings = list(skill_warnings)
    warnings.extend(resolution.warnings)
    warnings.extend(validation.warnings)

    speech = planner_result.get("speech", "")

    # planner 가 거절 대사를 만들지 못해도 validator 가 최종 사용자 메시지를 보수적으로 보정한다.
    if resolution.message_override:
        speech = resolution.message_override
    elif classifier_result.get("needs_motion", False) and not has_actionable_motion_command(validation.valid_commands):
        speech = build_motion_block_message(robot_state)
    elif user_text_requests_motion(user_text) and not has_actionable_motion_command(validation.valid_commands):
        speech = build_motion_block_message(robot_state)

    return ValidatedPlan(
        skills=list(planner_result.get("skills", [])),
        raw_commands=list(planner_result.get("commands", [])),
        expanded_commands=expanded_commands,
        resolved_commands=list(resolution.commands),
        valid_commands=list(validation.valid_commands),
        rejected_commands=list(validation.rejected_commands),
        warnings=warnings,
        speech=speech,
        reason=planner_result.get("reason", ""),
    )
