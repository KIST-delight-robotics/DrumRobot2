import re
from dataclasses import dataclass, field
from typing import List, Optional

try:
    from .command_validator import JOINT_LIMITS
except ImportError:
    from command_validator import JOINT_LIMITS

DEFAULT_RELATIVE_STEP_DEG = 15.0
JOINT_LIMIT_EPSILON_DEG = 0.1

JOINT_ALIASES = [
    ("왼쪽 손목", "L_wrist", "왼쪽 손목"),
    ("왼손목", "L_wrist", "왼쪽 손목"),
    ("오른쪽 손목", "R_wrist", "오른쪽 손목"),
    ("오른손목", "R_wrist", "오른쪽 손목"),
    ("왼쪽 발", "L_foot", "왼쪽 발"),
    ("왼발", "L_foot", "왼쪽 발"),
    ("오른쪽 발", "R_foot", "오른쪽 발"),
    ("오른발", "R_foot", "오른쪽 발"),
    ("허리", "waist", "허리"),
    ("왼쪽 팔목", "L_wrist", "왼쪽 손목"),
    ("오른쪽 팔목", "R_wrist", "오른쪽 손목"),
]

UP_KEYWORDS = ["올려", "올리", "들어", "높여", "위로"]
DOWN_KEYWORDS = ["내려", "내리", "낮춰", "아래로"]
ABSOLUTE_DEGREE_PATTERN = re.compile(r"(-?\d+(?:\.\d+)?)\s*도로")
RELATIVE_DEGREE_PATTERN = re.compile(r"(-?\d+(?:\.\d+)?)\s*도")


@dataclass
class MotionResolutionResult:
    commands: List[str]
    message_override: Optional[str] = None
    warnings: List[str] = field(default_factory=list)


def resolve_motion_commands(user_text, commands, robot_state):
    """
    상대 이동 표현을 현재 각도 기반 절대각 명령으로 변환한다.
    이 레이어는 planner를 본격 도입하기 전의 얇은 motion resolver 역할을 한다.
    """
    result = MotionResolutionResult(commands=list(commands))
    intent = parse_relative_motion_intent(user_text, robot_state=robot_state)
    if intent is None:
        return result

    current_angles = robot_state.get("current_angles", {})
    current_angle = current_angles.get(intent["joint_name"])
    if not isinstance(current_angle, (int, float)):
        result.message_override = f"{intent['display_name']}의 현재 각도를 아직 확인할 수 없어 지금은 해당 동작을 수행할 수 없습니다."
        result.warnings.append(f"현재 각도를 알 수 없어 상대 이동 해석 실패: {intent['joint_name']}")
        result.commands = _remove_move_commands(result.commands)
        return result

    delta = intent["delta_deg"] if intent["delta_deg"] is not None else DEFAULT_RELATIVE_STEP_DEG
    target_angle = float(current_angle) + (intent["direction"] * delta)
    min_angle, max_angle = JOINT_LIMITS[intent["joint_name"]]

    # 실측 각도가 75.03처럼 살짝 떠 있는 경우, 상대 이동 결과가 90.03이 되어
    # 물리적으로는 가능한 최대각(90도)까지의 동작까지 막히지 않도록 작은 오차는 경계값으로 클램프한다.
    if target_angle > max_angle and (target_angle - max_angle) <= JOINT_LIMIT_EPSILON_DEG:
        target_angle = max_angle
    elif target_angle < min_angle and (min_angle - target_angle) <= JOINT_LIMIT_EPSILON_DEG:
        target_angle = min_angle

    if target_angle < min_angle or target_angle > max_angle:
        direction_text = "올리면" if intent["direction"] > 0 else "내리면"
        result.message_override = (
            f"{intent['display_name']}은 현재 {float(current_angle):.1f}도에서 "
            f"{delta:.1f}도를 더 {direction_text} 한계 {min_angle:.1f}도에서 {max_angle:.1f}도를 벗어나 움직일 수 없습니다."
        )
        result.warnings.append(
            f"상대 이동 한계 초과 차단: {intent['joint_name']} {current_angle:.1f} -> {target_angle:.1f}"
        )
        result.commands = _remove_motion_sequence_commands(result.commands)
        return result

    resolved_command = _format_move_command(intent["joint_name"], target_angle)
    result.commands = _replace_or_append_move_command(result.commands, resolved_command)
    result.warnings.append(
        f"상대 이동 해석: {intent['joint_name']} {current_angle:.1f} -> {target_angle:.1f}"
    )
    return result


def parse_relative_motion_intent(user_text, robot_state=None):
    """
    사용자의 자연어에서 "현재 각도 기준 상대 이동" 의도를 추출한다.
    robot_state 가 있으면 명시적 관절이 없어도 last_action 기반 추론을 시도한다.
    """
    text = (user_text or "").strip()
    if not text:
        return None

    if ABSOLUTE_DEGREE_PATTERN.search(text):
        return None

    joint_info = _find_joint(text)
    if joint_info is None:
        joint_info = _infer_joint_from_context(text, robot_state)
    if joint_info is None:
        return None

    direction = _find_direction(text)
    if direction == 0:
        return None

    delta_match = RELATIVE_DEGREE_PATTERN.search(text)
    delta_deg = float(delta_match.group(1)) if delta_match else None

    return {
        "joint_name": joint_info["joint_name"],
        "display_name": joint_info["display_name"],
        "direction": direction,
        "delta_deg": delta_deg,
    }


def _find_joint(text):
    for alias, joint_name, display_name in JOINT_ALIASES:
        if alias in text:
            return {
                "joint_name": joint_name,
                "display_name": display_name,
            }
    return None


def _infer_joint_from_context(text, robot_state):
    """
    "거기서", "지금 상태에서", "더 올려" 같이 관절 이름이 생략된 경우
    최근 move 명령의 타겟 관절을 이어받는다.
    """
    if not text:
        return None

    contextual_keywords = ["거기서", "지금 상태", "지금", "더 ", "더올", "더 내려", "더 올려"]
    if not any(keyword in text for keyword in contextual_keywords):
        return None

    if not isinstance(robot_state, dict):
        return None

    last_action = robot_state.get("last_action", "")
    if not isinstance(last_action, str) or not last_action.startswith("move:"):
        return None

    try:
        payload = last_action.split(":", 1)[1]
        joint_name = payload.split(",", 1)[0]
    except (IndexError, ValueError):
        return None

    for _, candidate_joint_name, display_name in JOINT_ALIASES:
        if joint_name == candidate_joint_name:
            return {
                "joint_name": candidate_joint_name,
                "display_name": display_name,
            }

    return None


def _find_direction(text):
    if any(keyword in text for keyword in UP_KEYWORDS):
        return 1
    if any(keyword in text for keyword in DOWN_KEYWORDS):
        return -1
    return 0


def _format_move_command(joint_name, target_angle):
    if float(target_angle).is_integer():
        target_text = str(int(target_angle))
    else:
        target_text = f"{target_angle:.1f}"
    return f"move:{joint_name},{target_text}"


def _replace_or_append_move_command(commands, move_command):
    updated_commands = []
    replaced = False

    for command in commands:
        if command.startswith("move:") and not replaced:
            updated_commands.append(move_command)
            replaced = True
        elif not command.startswith("move:"):
            updated_commands.append(command)

    if not replaced:
        updated_commands.append(move_command)

    return updated_commands


def _remove_move_commands(commands):
    return [command for command in commands if not command.startswith("move:")]


def _remove_motion_sequence_commands(commands):
    """
    상대 이동 자체가 실패하면 그에 딸린 wait 도 같이 제거한다.
    사용자는 보통 "움직이고 기다리기"를 한 묶음 의도로 말하기 때문이다.
    """
    return [
        command
        for command in commands
        if not command.startswith("move:") and not command.startswith("wait:")
    ]
