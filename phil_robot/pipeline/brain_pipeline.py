import time
from dataclasses import dataclass, field
from typing import Dict, List

try:
    # 패키지 문맥으로 import될 때 사용한다.
    # 예: from phil_robot.pipeline.brain_pipeline import run_brain_turn
    from .intent_classifier import (
        # classifier system prompt
        CLASSIFIER_SYSTEM_PROMPT,
        # classifier 입력 payload 생성
        build_classifier_payload,
        # classifier 결과 후처리/정규화
        normalize_intent_result,
        # classifier JSON 응답 파싱
        parse_intent_response,
    )
    # JSON 형식 LLM 호출 공통 래퍼
    from .llm_interface import call_json_llm
    from .planner import (
        # planner 입력 payload 생성
        build_planner_payload,
        # planner 결과를 intent/domain 기준으로 한 번 더 정리
        enforce_intent_constraints,
        # domain별 planner system prompt 생성
        get_planner_system_prompt,
        # planner JSON 응답 파싱
        parse_plan_response,
        # classifier intent -> planner domain 변환
        select_planner_domain,
    )
    # raw state 정리 + 관절 각도 질문 shortcut 처리
    from .state_adapter import adapt_robot_state, build_joint_angle_answer, detect_joint_angle_query
    # planner 결과를 최종 실행 가능 계획으로 검증/정리
    from .validator import ValidatedPlan, build_validated_plan
except ImportError:
    # phil_robot 폴더 안에서 직접 실행할 때 사용하는 fallback import다.
    # 예: cd phil_robot && python phil_brain.py
    from pipeline.intent_classifier import (
        # classifier system prompt
        CLASSIFIER_SYSTEM_PROMPT,
        # classifier 입력 payload 생성
        build_classifier_payload,
        # classifier 결과 후처리/정규화
        normalize_intent_result,
        # classifier JSON 응답 파싱
        parse_intent_response,
    )
    # JSON 형식 LLM 호출 공통 래퍼
    from pipeline.llm_interface import call_json_llm
    from pipeline.planner import (
        # planner 입력 payload 생성
        build_planner_payload,
        # planner 결과를 intent/domain 기준으로 한 번 더 정리
        enforce_intent_constraints,
        # domain별 planner system prompt 생성
        get_planner_system_prompt,
        # planner JSON 응답 파싱
        parse_plan_response,
        # classifier intent -> planner domain 변환
        select_planner_domain,
    )
    # raw state 정리 + 관절 각도 질문 shortcut 처리
    from pipeline.state_adapter import adapt_robot_state, build_joint_angle_answer, detect_joint_angle_query
    # planner 결과를 최종 실행 가능 계획으로 검증/정리
    from pipeline.validator import ValidatedPlan, build_validated_plan

try:
    # 패키지 문맥 import일 때 공용 모델 설정을 가져온다.
    from ..config import CLASSIFIER_MODEL, PLANNER_MODEL
except ImportError:
    # 직접 실행 시에는 phil_robot 루트의 config 모듈에서 가져온다.
    from config import CLASSIFIER_MODEL, PLANNER_MODEL


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
    model_name=PLANNER_MODEL,
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
    classifier_result = normalize_intent_result(classifier_result, user_text)

    planner_domain = select_planner_domain(classifier_result)
    joint_angle_query = detect_joint_angle_query(user_text)

    # Shortcut path:
    # 특정 관절의 현재 각도를 묻는 상태 질의는 planner 를 거치지 않고
    # 현재 상태 스냅샷에서 직접 답해 더 빠르고 안정적으로 처리한다.
    if classifier_result.get("intent") == "status_question" and joint_angle_query:
        deterministic_speech = build_joint_angle_answer(adapted_state, joint_angle_query)
        planner_result = {
            "skills": [],
            "commands": [],
            "speech": deterministic_speech,
            "reason": "현재 관절 각도 조회는 상태 스냅샷에서 직접 응답",
        }
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
            prompt_payload="",
            classifier_raw_response_text=classifier_raw_response_text,
            raw_response_text="",
            planner_result=planner_result,
            adapted_state=adapted_state,
            validated_plan=validated_plan,
            classifier_duration_sec=classifier_duration_sec,
            planner_duration_sec=0.0,
            llm_duration_sec=classifier_duration_sec,
        )

    # Planner path:
    # 일반 대화/행동/연주 요청은 planner LLM 에게 위임해
    # skill/command/speech plan 을 만들고 validator 로 넘긴다.
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
