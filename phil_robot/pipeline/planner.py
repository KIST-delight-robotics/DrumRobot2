"""
2차 planner LLM 계층.

classifier 가 의도를 정리한 뒤, planner 는 그 결과를 바탕으로
speech 와 skill/command plan 만 생성한다.
"""

import json
import re
from typing import Dict, List, Set

try:
    from .llm_contract import FALLBACK_MESSAGE
    from .state_adapter import build_planner_state_summary
    from .skills import describe_skills_for_prompt, filter_skills_by_allowed_categories
except ImportError:
    from llm_contract import FALLBACK_MESSAGE
    from state_adapter import build_planner_state_summary
    from skills import describe_skills_for_prompt, filter_skills_by_allowed_categories


PLANNER_RESPONSE_SCHEMA_EXAMPLE = {
    "skills": ["wave_hi"],
    "op_cmd": [],
    "speech": "안녕하세요!",
    "reason": "simple greeting",
}

SKILL_CATALOG_TEXT = describe_skills_for_prompt()

PLANNER_DOMAIN_DEFAULT = "generic"
PLANNER_DOMAIN_CHAT = "chat"
PLANNER_DOMAIN_MOTION = "motion"
PLANNER_DOMAIN_PLAY = "play"
PLANNER_DOMAIN_STATUS = "status"
PLANNER_DOMAIN_STOP = "stop"

INTENT_TO_DOMAIN = {
    "chat": PLANNER_DOMAIN_CHAT,
    "motion_request": PLANNER_DOMAIN_MOTION,
    "play_request": PLANNER_DOMAIN_PLAY,
    "status_question": PLANNER_DOMAIN_STATUS,
    "stop_request": PLANNER_DOMAIN_STOP,
    "unknown": PLANNER_DOMAIN_DEFAULT,
}

DOMAIN_ALLOWED_SKILL_CATEGORIES: Dict[str, Set[str]] = {
    PLANNER_DOMAIN_CHAT: set(),
    PLANNER_DOMAIN_MOTION: {"social", "visual", "posture"},
    PLANNER_DOMAIN_PLAY: {"play", "posture"},
    PLANNER_DOMAIN_STATUS: set(),
    PLANNER_DOMAIN_STOP: {"posture", "system"},
    PLANNER_DOMAIN_DEFAULT: {"social", "visual", "posture", "play", "system"},
}

DOMAIN_INSTRUCTIONS = {
    PLANNER_DOMAIN_CHAT: """당신은 chat planner 다.
- 일반 대화, 인사, 잡담, 상식 질문에만 집중한다.
- 기본값은 speech 중심 응답이다.
- 특별히 로봇 동작이 꼭 필요하지 않으면 skills 와 op_cmd 는 빈 배열로 둔다.
- 상태 설명이나 제어 결정을 과장하지 말고 짧고 자연스럽게 응답한다.""",
    PLANNER_DOMAIN_MOTION: """당신은 motion planner 다.
- 손, 팔, 손목, 허리, 시선, 제스처 같은 물리 동작 요청에 집중한다.
- 가능한 경우 low-level command 보다 skill 을 우선 사용한다.
- 고개/시선 방향 요청은 가능하면 look_left/look_right/look_up/look_down/look_forward 같은 visual skill 을 우선 사용한다.
- skill 로 표현하기 어려운 세부 관절 제어만 op_cmd 에 직접 쓴다.
- 불가능하거나 unsafe 한 동작은 억지로 계획하지 말고 speech 를 통해 정중히 설명한다.""",
    PLANNER_DOMAIN_PLAY: """당신은 play planner 다.
- 곡 재생, 연주 시작, 준비 자세 같은 연주 관련 요청만 다룬다.
- 가능한 경우 play 관련 skill 을 우선 사용한다.
- 일반 social skill 이나 unrelated motion 은 넣지 않는다.
- speech 는 곡 소개와 실행 의도를 짧고 자연스럽게 전달한다.""",
    PLANNER_DOMAIN_STATUS: """당신은 status planner 다.
- 현재 상태, 직전 행동, 에러 원인, 왜 멈췄는지 같은 질문에 집중한다.
- 기본값은 speech 중심 응답이다.
- 상태 설명에 굳이 물리 동작을 붙이지 않는다.
- robot_state.current_angles 에 관절 각도 정보가 있으면, 특정 관절의 현재 각도 질문에 그 값을 직접 말해준다.
- 이름, 정체, 자기소개를 묻는 질문이 들어오면 현재 상태 설명보다 필의 정체성을 우선 소개한다.
- robot_state 에 직접 보이는 근거만 설명하고, current_song/progress 만으로 지금 연주 중이라고 추측하지 않는다. can_move=false 이고 busy=false 면 안전 키 상태를 먼저 설명한다.
- 사과, 설명, 안내를 명확하게 하되 장황하게 늘어놓지 않는다.""",
    PLANNER_DOMAIN_STOP: """당신은 stop planner 다.
- 멈춤, 정지, 종료, 홈 자세 복귀 요청에 집중한다.
- 가능한 경우 posture 또는 system skill 을 사용한다.
- speech 는 짧고 명확하게 현재 중단/종료 의도를 전달한다.
- unrelated motion 이나 social skill 은 넣지 않는다.""",
    PLANNER_DOMAIN_DEFAULT: """당신은 generic planner 다.
- 입력 의도가 불명확할 때는 보수적으로 행동한다.
- "왜?", "뭐?", "응?"처럼 짧고 맥락 없는 후속 발화는 이유를 추측하지 말고 무엇을 뜻하는지 짧게 되묻는다.
- 과도한 동작 계획보다 speech 중심으로 응답한다.
- 꼭 필요한 경우에만 op_cmd 또는 skills 를 생성한다.""",
}

PLANNER_SHARED_RULES = f"""반드시 JSON 객체 하나만 출력한다. 설명문, 코드블록, 마크다운은 절대 출력하지 않는다.

planner 입력에는 다음 정보가 함께 들어온다.
- robot_state: 현재 로봇 상태 요약
- intent_result: 1차 classifier 결과
- planner_domain: 현재 planner 도메인
- user_text: 사용자 발화

공통 규칙:
- 당신의 이름은 필(Phil)이며, KIST에서 개발된 지능형 휴머노이드 드럼 로봇이다.
- intent_result 를 반드시 따른다.
- intent_result.needs_motion 이 false 면 skills 와 op_cmd 를 모두 빈 배열로 둔다.
- 안전 키 잠김, 연주 중, 에러 상태, 이동 중이면 무리하게 동작 계획을 만들지 않는다.
- speech 는 TTS 용 자연스러운 한국어 문장만 쓴다. 괄호 설명문은 금지한다.
- move 명령은 move:L_wrist,90 처럼 실제 모터 이름을 바로 쓴다.
- look 명령 형식은 look:pan,tilt 이다. pan 은 좌우 회전이고 오른쪽은 양수, 왼쪽은 음수다. tilt 는 상하 각도이며 정면은 90, 위는 70 근처, 아래는 110 근처다.
- low-level move/look/wait 명령은 skill 로 표현하기 어려운 경우에만 op_cmd 에 직접 넣는다.

사용 가능한 skill 카탈로그:
{SKILL_CATALOG_TEXT}

사용 가능한 low-level command 예시:
- r
- h
- s
- look:0,90
- look:30,90
- look:-30,90
- look:0,70
- look:0,110
- gesture:wave
- led:happy
- move:L_wrist,90
- wait:2
- p:TIM

출력 스키마:
{{
  "skills": ["미리 정의된 skill 이름"],
  "op_cmd": ["skill 로 표현하기 어려운 low-level 명령"],
  "speech": "사용자에게 TTS 로 말할 문장",
  "reason": "planner 내부 판단 요약"
}}

출력 규칙:
- skills 는 문자열 배열이다. 없으면 [] 를 사용한다.
- op_cmd 는 문자열 배열이다. 없으면 [] 를 사용한다.
- speech 는 반드시 비어 있지 않은 문자열이어야 한다.
- reason 은 짧은 문자열이면 충분하다.
"""


def select_planner_domain(intent_result: Dict) -> str:
    """classifier intent 를 planner 도메인으로 변환한다."""
    intent = intent_result.get("intent", "unknown")
    if intent == "chat" and intent_result.get("needs_motion", False):
        return PLANNER_DOMAIN_MOTION
    return INTENT_TO_DOMAIN.get(intent, PLANNER_DOMAIN_DEFAULT)


def get_planner_system_prompt(planner_domain: str) -> str:
    """도메인별 planner system prompt 를 생성한다."""
    domain_instruction = DOMAIN_INSTRUCTIONS.get(planner_domain, DOMAIN_INSTRUCTIONS[PLANNER_DOMAIN_DEFAULT])
    return f"{domain_instruction}\n\n{PLANNER_SHARED_RULES}"


def build_planner_input_json(robot_state: Dict, user_text: str, intent_result: Dict, planner_domain: str) -> str:
    """planner 에 넘길 입력 JSON 문자열을 만든다."""
    state_summary = build_planner_state_summary(robot_state)
    return json.dumps(
        {
            "robot_state": state_summary,
            "intent_result": intent_result,
            "planner_domain": planner_domain,
            "user_text": user_text,
            "response_schema": PLANNER_RESPONSE_SCHEMA_EXAMPLE,
        },
        ensure_ascii=False,
        indent=2,
    )


def _sanitize_speech(speech: str) -> str:
    if not isinstance(speech, str):
        return FALLBACK_MESSAGE
    clean_msg = re.sub(r"\([^)]*\)", "", speech)
    sanitized = re.sub(r"\s+", " ", clean_msg).strip()
    return sanitized or FALLBACK_MESSAGE


def parse_plan_response(response_text: str) -> Dict:
    """
    planner JSON 응답을 읽어 skill/op_cmd/speech 구조로 정리한다.
    실패 시에도 이후 validator/executor 가 처리할 수 있는 기본형을 반환한다.
    """
    result = {
        "skills": [],
        "op_cmd": [],
        "speech": FALLBACK_MESSAGE,
        "reason": "",
    }

    if not isinstance(response_text, str):
        return result

    try:
        response_data = json.loads(response_text)
    except json.JSONDecodeError:
        return result

    if not isinstance(response_data, dict):
        return result

    raw_skills = response_data.get("skills", [])
    raw_op_cmds = response_data.get("op_cmd", response_data.get("commands", []))
    raw_speech = response_data.get("speech", FALLBACK_MESSAGE)
    raw_reason = response_data.get("reason", "")

    if isinstance(raw_skills, list):
        result["skills"] = [skill.strip() for skill in raw_skills if isinstance(skill, str) and skill.strip()]
    if isinstance(raw_op_cmds, list):
        result["op_cmd"] = [cmd.strip() for cmd in raw_op_cmds if isinstance(cmd, str) and cmd.strip()]
    result["speech"] = _sanitize_speech(raw_speech)
    if isinstance(raw_reason, str):
        result["reason"] = raw_reason.strip()

    return result


def enforce_intent_constraints(planner_result: Dict, intent_result: Dict) -> Dict:
    """
    classifier 결과를 planner 뒤에서도 한 번 더 강제한다.
    planner 가 습관적으로 gesture/look 를 붙여도 여기서 정리한다.
    """
    normalized = {
        "skills": list(planner_result.get("skills", [])),
        "op_cmd": list(planner_result.get("op_cmd", planner_result.get("commands", []))),
        "speech": planner_result.get("speech", FALLBACK_MESSAGE),
        "reason": planner_result.get("reason", ""),
    }

    intent = intent_result.get("intent", "unknown")
    needs_motion = intent_result.get("needs_motion", False)

    if not needs_motion:
        normalized["skills"] = []
        normalized["op_cmd"] = []
        return normalized

    planner_domain = select_planner_domain(intent_result)
    allowed_categories = DOMAIN_ALLOWED_SKILL_CATEGORIES.get(planner_domain, set())
    if allowed_categories:
        normalized["skills"] = filter_skills_by_allowed_categories(
            normalized["skills"],
            allowed_categories,
        )

    if intent == "play_request":
        allowed_prefixes = ("r", "p:", "led:", "wait:")
        normalized["op_cmd"] = [
            command for command in normalized["op_cmd"] if command.startswith(allowed_prefixes)
        ]
    elif intent == "stop_request":
        allowed_prefixes = ("h", "s", "led:", "wait:")
        normalized["op_cmd"] = [
            command for command in normalized["op_cmd"] if command.startswith(allowed_prefixes)
        ]
    elif intent == "motion_request":
        allowed_prefixes = ("move:", "look:", "gesture:", "led:", "wait:", "r", "h")
        normalized["op_cmd"] = [
            command for command in normalized["op_cmd"] if command.startswith(allowed_prefixes)
        ]
    elif intent == "status_question":
        normalized["skills"] = []
        normalized["op_cmd"] = []
    elif intent == "chat" and not needs_motion:
        normalized["skills"] = []
        normalized["op_cmd"] = []

    return normalized
