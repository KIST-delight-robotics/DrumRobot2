"""
plan 단위 executor 레이어.

validator 를 통과한 plan 만 실제 로봇으로 전송한다.
"""

from dataclasses import dataclass, field
from typing import List

try:
    from .command_executor import execute_commands
    from .validator import ValidatedPlan
except ImportError:
    from command_executor import execute_commands
    from validator import ValidatedPlan


@dataclass
class ExecutionResult:
    requested_op_cmds: List[str] = field(default_factory=list)
    requested_transport_cmds: List[str] = field(default_factory=list)
    executed_transport_cmds: List[str] = field(default_factory=list)


def build_transport_commands(validated_plan: ValidatedPlan) -> List[str]:
    """
    plan 레벨 명령을 실제 TCP 전송 문자열 목록으로 펼친다.
    play modifier 는 play command 보다 먼저 보내 next-play 설정처럼 동작하게 둔다.
    """
    transport_cmds: List[str] = []

    if validated_plan.play_modifier is not None:
        if validated_plan.play_modifier.tempo_scale != 1.0:
            transport_cmds.append(f"tempo_scale:{validated_plan.play_modifier.tempo_scale:.2f}")
        if validated_plan.play_modifier.velocity_delta != 0:
            transport_cmds.append(f"velocity_delta:{validated_plan.play_modifier.velocity_delta}")

    transport_cmds.extend(list(validated_plan.valid_op_cmds))
    return transport_cmds


def execute_validated_plan(bot, validated_plan: ValidatedPlan) -> ExecutionResult:
    """
    executor 는 validator 가 통과시킨 최종 명령만 소비한다.
    이 함수는 나중에 retry, rollback, ACK 추적을 붙일 자리를 남겨둔다.
    """
    requested_op_cmds = list(validated_plan.valid_op_cmds)
    requested_transport_cmds = build_transport_commands(validated_plan)
    executed_transport_cmds = execute_commands(bot, requested_transport_cmds)
    return ExecutionResult(
        requested_op_cmds=requested_op_cmds,
        requested_transport_cmds=requested_transport_cmds,
        executed_transport_cmds=executed_transport_cmds,
    )
