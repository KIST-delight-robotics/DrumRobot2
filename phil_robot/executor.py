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
    requested_commands: List[str] = field(default_factory=list)
    executed_commands: List[str] = field(default_factory=list)


def execute_validated_plan(bot, validated_plan: ValidatedPlan) -> ExecutionResult:
    """
    executor 는 validator 가 통과시킨 최종 명령만 소비한다.
    이 함수는 나중에 retry, rollback, ACK 추적을 붙일 자리를 남겨둔다.
    """
    requested_commands = list(validated_plan.valid_commands)
    executed_commands = execute_commands(bot, requested_commands)
    return ExecutionResult(
        requested_commands=requested_commands,
        executed_commands=executed_commands,
    )
