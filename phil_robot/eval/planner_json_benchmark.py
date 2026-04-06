import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from phil_robot.config import CLASSIFIER_MODEL, PLANNER_MODEL
from phil_robot.pipeline.failure import FALLBACK_MESSAGE
from phil_robot.pipeline.intent_classifier import (
    CLASSIFIER_SYSTEM_PROMPT,
    build_classifier_input_json,
    is_ambiguous_follow_up,
    normalize_intent_result,
    parse_intent_response,
)
from phil_robot.pipeline.llm_interface import call_json_llm
from phil_robot.pipeline.planner import (
    build_planner_input_json,
    enforce_intent_constraints,
    get_planner_system_prompt,
    parse_plan_response,
    select_planner_domain,
)
from phil_robot.pipeline.state_adapter import adapt_robot_state, build_joint_angle_answer, detect_joint_angle_query
from phil_robot.pipeline.validator import build_validated_plan


@dataclass
class PreparedPlannerCase:
    id: str
    tags: List[str] = field(default_factory=list)
    user_text: str = ""
    expected: Dict[str, Any] = field(default_factory=dict)
    adapted_state: Dict[str, Any] = field(default_factory=dict)
    classifier_input_json: str = ""
    classifier_raw_response_text: str = ""
    classifier_result: Dict[str, Any] = field(default_factory=dict)
    classifier_metrics: Dict[str, Any] = field(default_factory=dict)
    planner_domain: str = ""
    planner_system_prompt: str = ""
    planner_input_json: str = ""
    planner_enabled: bool = True
    shortcut_reason: str = ""
    shortcut_plan: Dict[str, Any] = field(default_factory=dict)


def loads_json_obj(raw_text: str) -> bool:
    if not isinstance(raw_text, str) or not raw_text.strip():
        return False

    try:
        json_obj = json.loads(raw_text)
    except json.JSONDecodeError:
        return False

    return isinstance(json_obj, dict)


def median_num(num_list: List[float]) -> Optional[float]:
    if not num_list:
        return None

    sorted_list = sorted(num_list)
    size_num = len(sorted_list)
    mid_num = size_num // 2
    if size_num % 2 == 1:
        return sorted_list[mid_num]
    return (sorted_list[mid_num - 1] + sorted_list[mid_num]) / 2.0


def p95_num(num_list: List[float]) -> Optional[float]:
    if not num_list:
        return None

    sorted_list = sorted(num_list)
    idx_num = int(len(sorted_list) * 0.95)
    if idx_num < 0:
        idx_num = 0
    if idx_num >= len(sorted_list):
        idx_num = len(sorted_list) - 1
    return sorted_list[idx_num]


def _build_shortcut_plan(user_text: str, adapted_state: Dict[str, Any], classifier_result: Dict[str, Any]) -> Tuple[bool, str, Dict[str, Any]]:
    if classifier_result.get("intent") == "unknown" and is_ambiguous_follow_up(user_text):
        return (
            True,
            "ambiguous_follow_up",
            {
                "skills": [],
                "op_cmd": [],
                "speech": "무엇이 왜 그런지 조금만 더 구체적으로 말씀해 주세요.",
                "reason": "맥락 없는 초단문 후속 질문은 clarification 으로 직접 응답",
            },
        )

    joint_name = detect_joint_angle_query(user_text)
    if classifier_result.get("intent") == "status_question" and joint_name:
        return (
            True,
            "joint_angle_shortcut",
            {
                "skills": [],
                "op_cmd": [],
                "speech": build_joint_angle_answer(adapted_state, joint_name),
                "reason": "현재 관절 각도 조회는 상태 스냅샷에서 직접 응답",
            },
        )

    return False, "", {}


def prepare_planner_case(
    case: Dict[str, Any],
    classifier_name: str = CLASSIFIER_MODEL,
    capture_classifier_metrics: bool = False,
) -> PreparedPlannerCase:
    user_text = case["user_text"]
    adapted_state = adapt_robot_state(case["robot_state"])
    classifier_input_json = build_classifier_input_json(adapted_state, user_text)

    if capture_classifier_metrics:
        classifier_raw_response_text, classifier_metrics = call_json_llm(
            model_name=classifier_name,
            system_prompt=CLASSIFIER_SYSTEM_PROMPT,
            user_input_json=classifier_input_json,
            capture_metrics=True,
        )
    else:
        classifier_raw_response_text = call_json_llm(
            model_name=classifier_name,
            system_prompt=CLASSIFIER_SYSTEM_PROMPT,
            user_input_json=classifier_input_json,
        )
        classifier_metrics = {}

    classifier_result = parse_intent_response(classifier_raw_response_text)
    classifier_result = normalize_intent_result(classifier_result, user_text)
    planner_domain = select_planner_domain(classifier_result)
    shortcut_hit, shortcut_reason, shortcut_plan = _build_shortcut_plan(user_text, adapted_state, classifier_result)
    planner_enabled = not shortcut_hit

    if planner_enabled:
        planner_system_prompt = get_planner_system_prompt(planner_domain)
        planner_input_json = build_planner_input_json(adapted_state, user_text, classifier_result, planner_domain)
    else:
        planner_system_prompt = ""
        planner_input_json = ""

    return PreparedPlannerCase(
        id=case["id"],
        tags=list(case.get("tags", [])),
        user_text=user_text,
        expected=dict(case.get("expected", {})),
        adapted_state=adapted_state,
        classifier_input_json=classifier_input_json,
        classifier_raw_response_text=classifier_raw_response_text,
        classifier_result=classifier_result,
        classifier_metrics=dict(classifier_metrics),
        planner_domain=planner_domain,
        planner_system_prompt=planner_system_prompt,
        planner_input_json=planner_input_json,
        planner_enabled=planner_enabled,
        shortcut_reason=shortcut_reason,
        shortcut_plan=shortcut_plan,
    )


def prepare_planner_cases(
    case_list: List[Dict[str, Any]],
    classifier_name: str = CLASSIFIER_MODEL,
    capture_classifier_metrics: bool = False,
) -> List[PreparedPlannerCase]:
    return [
        prepare_planner_case(
            case_obj,
            classifier_name=classifier_name,
            capture_classifier_metrics=capture_classifier_metrics,
        )
        for case_obj in case_list
    ]


def execute_prepared_case(
    prepared_case: PreparedPlannerCase,
    planner_name: str = PLANNER_MODEL,
    capture_metrics: bool = False,
) -> Dict[str, Any]:
    planner_metrics: Dict[str, Any] = {}
    planner_duration_sec = 0.0

    if prepared_case.planner_enabled:
        start_sec = time.time()
        if capture_metrics:
            planner_raw_response_text, planner_metrics = call_json_llm(
                model_name=planner_name,
                system_prompt=prepared_case.planner_system_prompt,
                user_input_json=prepared_case.planner_input_json,
                capture_metrics=True,
            )
        else:
            planner_raw_response_text = call_json_llm(
                model_name=planner_name,
                system_prompt=prepared_case.planner_system_prompt,
                user_input_json=prepared_case.planner_input_json,
            )
        planner_duration_sec = time.time() - start_sec
        planner_result = parse_plan_response(planner_raw_response_text)
        planner_result = enforce_intent_constraints(planner_result, prepared_case.classifier_result)
        planner_parse_ok = loads_json_obj(planner_raw_response_text)
        planner_is_fallback = (
            not planner_parse_ok
            or planner_result.get("speech") == FALLBACK_MESSAGE
            or str(planner_result.get("reason", "")).startswith("LLM call failed:")
        )
    else:
        planner_raw_response_text = ""
        planner_result = dict(prepared_case.shortcut_plan)
        planner_parse_ok = True
        planner_is_fallback = False

    validated_plan = build_validated_plan(
        user_text=prepared_case.user_text,
        robot_state=prepared_case.adapted_state,
        classifier_result=prepared_case.classifier_result,
        planner_result=planner_result,
    )

    actual = {
        "intent": prepared_case.classifier_result.get("intent"),
        "needs_motion": prepared_case.classifier_result.get("needs_motion"),
        "needs_dialogue": prepared_case.classifier_result.get("needs_dialogue"),
        "risk_level": prepared_case.classifier_result.get("risk_level"),
        "planner_domain": prepared_case.planner_domain,
        "skills": list(validated_plan.skills),
        "raw_op_cmds": list(validated_plan.raw_op_cmds),
        "expanded_op_cmds": list(validated_plan.expanded_op_cmds),
        "resolved_op_cmds": list(validated_plan.resolved_op_cmds),
        "valid_op_cmds": list(validated_plan.valid_op_cmds),
        "rejected_op_cmds": list(validated_plan.rejected_op_cmds),
        "speech": validated_plan.speech,
        "reason": validated_plan.reason,
        "classifier_raw_response_text": prepared_case.classifier_raw_response_text,
        "planner_raw_response_text": planner_raw_response_text,
        "planner_called": prepared_case.planner_enabled,
        "planner_parse_ok": planner_parse_ok,
        "planner_is_fallback": planner_is_fallback,
    }

    return {
        "actual": actual,
        "planner_result": planner_result,
        "validated_plan": validated_plan,
        "planner_duration_sec": planner_duration_sec,
        "planner_metrics": dict(planner_metrics),
        "planner_input_chars": len(prepared_case.planner_input_json),
        "planner_response_chars": len(planner_raw_response_text) if isinstance(planner_raw_response_text, str) else 0,
    }
