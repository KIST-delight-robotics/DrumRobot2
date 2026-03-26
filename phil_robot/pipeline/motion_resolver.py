import re
from dataclasses import dataclass, field
from typing import List, Optional

try:
    from .command_validator import JOINT_LIMITS
except ImportError:
    from command_validator import JOINT_LIMITS

DEFAULT_RELATIVE_STEP_DEG = 15.0
JOINT_LIMIT_EPSILON_DEG = 0.1
LOOK_FORWARD_PAN_DEG = 0.0
LOOK_FORWARD_TILT_DEG = 90.0
LOOK_SIDE_PAN_DEG = 30.0
LOOK_UP_TILT_DEG = 70.0
LOOK_DOWN_TILT_DEG = 110.0

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
HEAD_DIRECTION_KEYWORDS = ["고개", "머리", "시선", "얼굴", "정면"]
LOOK_RIGHT_KEYWORDS = ["오른쪽", "오른편", "우측"]
LOOK_LEFT_KEYWORDS = ["왼쪽", "왼편", "좌측"]
LOOK_FORWARD_KEYWORDS = ["정면", "앞", "앞쪽"]
LOOK_UP_KEYWORDS = ["위", "위쪽", "위로", "올려다", "쳐다"]
LOOK_DOWN_KEYWORDS = ["아래", "아래쪽", "아래로", "내려다", "숙여"]
ABSOLUTE_DEGREE_PATTERN = re.compile(r"(-?\d+(?:\.\d+)?)\s*도로")
RELATIVE_DEGREE_PATTERN = re.compile(r"(-?\d+(?:\.\d+)?)\s*도")


@dataclass
class MotionResolutionResult:
    op_cmds: List[str]
    message_override: Optional[str] = None
    warnings: List[str] = field(default_factory=list)


def resolve_motion_commands(user_text, op_cmds, robot_state):
    """
    상대 이동 표현을 현재 각도 기반 절대각 명령으로 변환한다.
    이 레이어는 planner를 본격 도입하기 전의 얇은 motion resolver 역할을 한다.
    """
    result = MotionResolutionResult(op_cmds=list(op_cmds))

    look_op_cmd = parse_head_look_command(user_text)
    if look_op_cmd is not None:
        result.op_cmds = _replace_or_append_look_op_cmd(result.op_cmds, look_op_cmd)
        result.warnings.append(f"고개 방향 해석: {look_op_cmd}")

    intent = parse_relative_motion_intent(user_text, robot_state=robot_state)
    if intent is None:
        return result

    current_angles = robot_state.get("current_angles", {})
    current_angle = current_angles.get(intent["joint_name"])
    if not isinstance(current_angle, (int, float)):
        result.message_override = f"{intent['display_name']}의 현재 각도를 아직 확인할 수 없어 지금은 해당 동작을 수행할 수 없습니다."
        result.warnings.append(f"현재 각도를 알 수 없어 상대 이동 해석 실패: {intent['joint_name']}")
        result.op_cmds = _remove_move_op_cmds(result.op_cmds)
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
        result.op_cmds = _remove_motion_sequence_op_cmds(result.op_cmds)
        return result

    resolved_op_cmd = _format_move_command(intent["joint_name"], target_angle)
    result.op_cmds = _replace_or_append_move_op_cmd(result.op_cmds, resolved_op_cmd)
    result.warnings.append(
        f"상대 이동 해석: {intent['joint_name']} {current_angle:.1f} -> {target_angle:.1f}"
    )
    return result


def parse_head_look_command(user_text):
    """
    고개/시선 방향 요청을 look:pan,tilt 명령으로 정규화한다.
    LLM 이 pan/tilt 축을 헷갈려도 이 레이어에서 의미를 바로잡는다.
    """
    text = (user_text or "").strip()
    if not text:
        return None

    if not any(keyword in text for keyword in HEAD_DIRECTION_KEYWORDS):
        return None

    pan_deg = None
    tilt_deg = None

    if any(keyword in text for keyword in LOOK_RIGHT_KEYWORDS):
        pan_deg = LOOK_SIDE_PAN_DEG
    elif any(keyword in text for keyword in LOOK_LEFT_KEYWORDS):
        pan_deg = -LOOK_SIDE_PAN_DEG
    elif any(keyword in text for keyword in LOOK_FORWARD_KEYWORDS):
        pan_deg = LOOK_FORWARD_PAN_DEG

    if any(keyword in text for keyword in LOOK_UP_KEYWORDS):
        tilt_deg = LOOK_UP_TILT_DEG
    elif any(keyword in text for keyword in LOOK_DOWN_KEYWORDS):
        tilt_deg = LOOK_DOWN_TILT_DEG

    if pan_deg is None and tilt_deg is None:
        return None

    if pan_deg is None:
        pan_deg = LOOK_FORWARD_PAN_DEG
    if tilt_deg is None:
        tilt_deg = LOOK_FORWARD_TILT_DEG

    return _format_look_command(pan_deg, tilt_deg)


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
        action_args = last_action.split(":", 1)[1]
        joint_name = action_args.split(",", 1)[0]
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


def _format_look_command(pan_deg, tilt_deg):
    if float(pan_deg).is_integer():
        pan_text = str(int(pan_deg))
    else:
        pan_text = f"{pan_deg:.1f}"

    if float(tilt_deg).is_integer():
        tilt_text = str(int(tilt_deg))
    else:
        tilt_text = f"{tilt_deg:.1f}"

    return f"look:{pan_text},{tilt_text}"


def _replace_or_append_move_op_cmd(op_cmds, move_op_cmd):
    updated_commands = []
    replaced = False

    for command in op_cmds:
        if command.startswith("move:") and not replaced:
            updated_commands.append(move_op_cmd)
            replaced = True
        elif not command.startswith("move:"):
            updated_commands.append(command)

    if not replaced:
        updated_commands.append(move_op_cmd)

    return updated_commands


def _replace_or_append_look_op_cmd(op_cmds, look_op_cmd):
    updated_commands = []
    replaced = False

    for command in op_cmds:
        if command.startswith("look:") and not replaced:
            updated_commands.append(look_op_cmd)
            replaced = True
        elif not command.startswith("look:"):
            updated_commands.append(command)

    if not replaced:
        updated_commands.append(look_op_cmd)

    return updated_commands


def _remove_move_op_cmds(op_cmds):
    return [command for command in op_cmds if not command.startswith("move:")]


def _remove_motion_sequence_op_cmds(op_cmds):
    """
    상대 이동 자체가 실패하면 그에 딸린 wait 도 같이 제거한다.
    사용자는 보통 "움직이고 기다리기"를 한 묶음 의도로 말하기 때문이다.
    """
    return [
        command
        for command in op_cmds
        if not command.startswith("move:") and not command.startswith("wait:")
    ]
