import time
from dataclasses import dataclass, field
from typing import Dict, List

try:
    from .intent_classifier import (
        CLASSIFIER_MODEL,
        CLASSIFIER_SYSTEM_PROMPT,
        build_classifier_payload,
        parse_intent_response,
    )
    from .llm_contract import DEFAULT_LLM_MODEL
    from .llm_interface import call_json_llm
    from .planner import (
        build_planner_payload,
        enforce_intent_constraints,
        get_planner_system_prompt,
        parse_plan_response,
        select_planner_domain,
    )
    from .state_adapter import adapt_robot_state
    from .validator import ValidatedPlan, build_validated_plan
except ImportError:
    from intent_classifier import (
        CLASSIFIER_MODEL,
        CLASSIFIER_SYSTEM_PROMPT,
        build_classifier_payload,
        parse_intent_response,
    )
    from llm_contract import DEFAULT_LLM_MODEL
    from llm_interface import call_json_llm
    from planner import (
        build_planner_payload,
        enforce_intent_constraints,
        get_planner_system_prompt,
        parse_plan_response,
        select_planner_domain,
    )
    from state_adapter import adapt_robot_state
    from validator import ValidatedPlan, build_validated_plan


@dataclass
class BrainTurnResult:
    classifier_payload: str
    classifier_result: Dict
    planner_domain: str
    prompt_payload: str
    classifier_raw_response_text: str
    raw_response_text: str
    planner_result: Dict
    adapted_state: Dict
    validated_plan: ValidatedPlan = field(default_factory=ValidatedPlan)
    classifier_duration_sec: float = 0.0
    planner_duration_sec: float = 0.0
    llm_duration_sec: float = 0.0


def run_brain_turn(
    user_text,
    raw_robot_state,
    model_name=DEFAULT_LLM_MODEL,
    classifier_model_name=CLASSIFIER_MODEL,
):
    """
    한 턴의 LLM 처리 파이프라인.
    1차 classifier 와 2차 planner 를 분리해, 각 역할을 독립적으로 다룰 수 있게 한다.
    """
    adapted_state = adapt_robot_state(raw_robot_state)

    classifier_payload = build_classifier_payload(adapted_state, user_text)
    classifier_start_time = time.time()
    classifier_raw_response_text = call_json_llm(
        model_name=classifier_model_name,
        system_prompt=CLASSIFIER_SYSTEM_PROMPT,
        user_payload=classifier_payload,
    )
    classifier_duration_sec = time.time() - classifier_start_time
    classifier_result = parse_intent_response(classifier_raw_response_text)

    planner_domain = select_planner_domain(classifier_result)
    planner_system_prompt = get_planner_system_prompt(planner_domain)
    prompt_payload = build_planner_payload(adapted_state, user_text, classifier_result, planner_domain)
    planner_start_time = time.time()
    raw_response_text = call_json_llm(
        model_name=model_name,
        system_prompt=planner_system_prompt,
        user_payload=prompt_payload,
    )
    planner_duration_sec = time.time() - planner_start_time

    planner_result = parse_plan_response(raw_response_text)
    planner_result = enforce_intent_constraints(planner_result, classifier_result)
    validated_plan = build_validated_plan(
        user_text=user_text,
        robot_state=adapted_state,
        classifier_result=classifier_result,
        planner_result=planner_result,
    )

    return BrainTurnResult(
        classifier_payload=classifier_payload,
        classifier_result=classifier_result,
        planner_domain=planner_domain,
        prompt_payload=prompt_payload,
        classifier_raw_response_text=classifier_raw_response_text,
        raw_response_text=raw_response_text,
        planner_result=planner_result,
        adapted_state=adapted_state,
        validated_plan=validated_plan,
        classifier_duration_sec=classifier_duration_sec,
        planner_duration_sec=planner_duration_sec,
        llm_duration_sec=classifier_duration_sec + planner_duration_sec,
    )
