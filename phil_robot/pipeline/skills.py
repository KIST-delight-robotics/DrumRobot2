"""
재사용 가능한 상위 동작(skill)을 정의한다.

LLM이 매번 low-level command 문자열을 직접 조립하지 않아도 되도록,
자주 쓰는 동작 시퀀스를 코드에 고정해 안정성을 높인다.
"""

from typing import Dict, List, Set, Tuple


SKILL_LIBRARY: Dict[str, Dict[str, object]] = {
    # 사회적 반응 / 인사
    "wave_hi": {
        "category": "social",
        "description": "정면을 보고 손을 흔들며 밝게 인사한다.",
        "op_cmd": ["look:0,90", "gesture:wave", "led:happy"],
    },
    "nod_yes": {
        "category": "social",
        "description": "가볍게 끄덕이며 긍정 반응을 보인다.",
        "op_cmd": ["gesture:nod"],
    },
    "shake_no": {
        "category": "social",
        "description": "고개를 좌우로 흔들며 부정 반응을 보인다.",
        "op_cmd": ["gesture:shake"],
    },
    "happy_react": {
        "category": "social",
        "description": "기쁜 제스처와 표정으로 반응한다.",
        "op_cmd": ["gesture:happy", "led:happy"],
    },
    "celebrate": {
        "category": "social",
        "description": "만세 동작으로 크게 기쁨을 표현한다.",
        "op_cmd": ["gesture:hurray", "led:happy"],
    },
    # 시선 계열
    "look_forward": {
        "category": "visual",
        "description": "정면을 바라본다.",
        "op_cmd": ["look:0,90"],
    },
    "look_left": {
        "category": "visual",
        "description": "왼쪽을 바라본다.",
        "op_cmd": ["look:-30,90"],
    },
    "look_right": {
        "category": "visual",
        "description": "오른쪽을 바라본다.",
        "op_cmd": ["look:30,90"],
    },
    "look_up": {
        "category": "visual",
        "description": "위쪽을 바라본다.",
        "op_cmd": ["look:0,70"],
    },
    "look_down": {
        "category": "visual",
        "description": "아래쪽을 바라본다.",
        "op_cmd": ["look:0,110"],
    },
    # 자세 / 전환
    "arm_up": {
        "category": "posture",
        "description": "양팔을 들어 올린다.",
        "op_cmd": ["move:R_arm2,70", "move:L_arm2,70", "move:R_arm3,15", "move:L_arm3,15"],
    },
    "arm_down": {
        "category": "posture",
        "description": "양팔을 아래 자세로 내린다.",
        "op_cmd": ["move:R_arm2,0", "move:L_arm2,0", "move:R_arm3,20", "move:L_arm3,20"],
    },
    "left_arm_up": {
        "category": "posture",
        "description": "왼팔을 들어 올린다.",
        "op_cmd": ["move:L_arm2,70", "move:L_arm3,15"],
    },
    "right_arm_up": {
        "category": "posture",
        "description": "오른팔을 들어 올린다.",
        "op_cmd": ["move:R_arm2,70", "move:R_arm3,15"],
    },
    "left_arm_down": {
        "category": "posture",
        "description": "왼팔을 아래 자세로 내린다.",
        "op_cmd": ["move:L_arm2,0", "move:L_arm3,20"],
    },
    "right_arm_down": {
        "category": "posture",
        "description": "오른팔을 아래 자세로 내린다.",
        "op_cmd": ["move:R_arm2,0", "move:R_arm3,20"],
    },
    "arms_out": {
        "category": "posture",
        "description": "양팔을 옆으로 벌린다.",
        "op_cmd": ["move:R_arm1,30", "move:L_arm1,150"],
    },
    "left_arm_out": {
        "category": "posture",
        "description": "왼팔을 옆으로 벌린다.",
        "op_cmd": ["move:L_arm1,150"],
    },
    "right_arm_out": {
        "category": "posture",
        "description": "오른팔을 옆으로 벌린다.",
        "op_cmd": ["move:R_arm1,30"],
    },
    "ready_pose": {
        "category": "posture",
        "description": "연주 준비 자세로 전환한다.",
        "op_cmd": ["r"],
    },
    "idle_home": {
        "category": "posture",
        "description": "휴식 자세로 돌아가 표정을 idle 로 맞춘다.",
        "op_cmd": ["h", "led:idle"],
    },
    # 연주 묶음
    "play_tim": {
        "category": "play",
        "description": "This Is Me 연주를 시작한다.",
        "op_cmd": ["r", "p:TIM", "led:play"],
    },
    "play_ty_short": {
        "category": "play",
        "description": "그대에게 연주를 시작한다.",
        "op_cmd": ["r", "p:TY_short", "led:play"],
    },
    "play_bi": {
        "category": "play",
        "description": "Baby I Need You 연주를 시작한다.",
        "op_cmd": ["r", "p:BI", "led:play"],
    },
    "play_test_one": {
        "category": "play",
        "description": "테스트 비트를 연주한다.",
        "op_cmd": ["r", "p:test_one", "led:play"],
    },
    # 시스템
    "shutdown_system": {
        "category": "system",
        "description": "시스템 종료 명령을 수행한다.",
        "op_cmd": ["s"],
    },
}


def list_skill_names() -> List[str]:
    """프롬프트에 노출할 수 있는 skill 이름 목록을 반환한다."""
    return sorted(SKILL_LIBRARY.keys())


def describe_skills_for_prompt() -> str:
    """planner 프롬프트에 넣기 쉬운 skill 카탈로그 문자열을 만든다."""
    lines: List[str] = []
    for skill_name in sorted(SKILL_LIBRARY.keys()):
        metadata = SKILL_LIBRARY[skill_name]
        lines.append(
            f"- {skill_name} ({metadata['category']}): {metadata['description']}"
        )
    return "\n".join(lines)


def get_skill_categories(skill_names: List[str]) -> Set[str]:
    """skill 이름 목록에 포함된 카테고리 집합을 반환한다."""
    categories: Set[str] = set()
    for skill_name in skill_names:
        metadata = SKILL_LIBRARY.get(skill_name)
        if metadata:
            categories.add(str(metadata.get("category", "")))
    return categories


def filter_skills_by_allowed_categories(skill_names: List[str], allowed_categories: Set[str]) -> List[str]:
    """허용 카테고리에 맞는 skill 만 남긴다."""
    filtered: List[str] = []
    for skill_name in skill_names:
        metadata = SKILL_LIBRARY.get(skill_name)
        if not metadata:
            continue
        if metadata.get("category") in allowed_categories:
            filtered.append(skill_name)
    return filtered


def expand_skills(skill_names: List[str]) -> Tuple[List[str], List[str]]:
    """
    planner가 고른 skill 이름을 실제 로봇 명령 시퀀스로 펼친다.

    반환값:
    - op_cmds: 전송 가능한 문자열 명령 목록
    - warnings: 알 수 없는 skill 등 디버그용 경고
    """
    op_cmds: List[str] = []
    warnings: List[str] = []

    for skill_name in skill_names:
        metadata = SKILL_LIBRARY.get(skill_name)
        if metadata is None:
            warnings.append(f"알 수 없는 skill 무시: {skill_name}")
            continue
        op_cmds.extend(list(metadata["op_cmd"]))

    return _deduplicate_consecutive_op_cmds(op_cmds), warnings


def _deduplicate_consecutive_op_cmds(op_cmds: List[str]) -> List[str]:
    """
    skill 조합 과정에서 동일한 명령이 연속으로 생기면 한 번만 남긴다.
    예: ready_pose + play_tim -> r, r, p:TIM -> r, p:TIM
    """
    deduped: List[str] = []
    for op_cmd in op_cmds:
        if deduped and deduped[-1] == op_cmd:
            continue
        deduped.append(op_cmd)
    return deduped
