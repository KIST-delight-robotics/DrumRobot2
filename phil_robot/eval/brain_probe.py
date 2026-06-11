"""
[eval/benchmark 전용] run_brain_turn / BrainTurnResult.

런타임 FSM(`pipeline/robot_fsm.py`)은 `pipeline/brain_pipeline.py` 의 step 함수
(build_prefilter_plan / classify_step / build_direct_answer_plan / planner_step)를
run_turn 에서 직접 호출하고 PhilState 만 굴린다. BrainTurnResult 같은 진단 래퍼는 쓰지 않는다.

이 모듈은 eval/benchmark 가 한 번의 호출로 진단 데이터(입력 JSON, raw 응답, latency,
metrics)를 모으기 위한 어댑터다. 런타임과 같은 step 함수를 호출하므로 결과가 일치한다.
즉, "엔진(step 함수)은 하나, 입구만 둘(런타임 FSM / eval probe)" 구조다.

주의: 이 경로는 single-turn 진단/벤치마크용이다. multi-turn 대화/복구(recovery thread)는
PhilState 를 턴마다 invoke 하는 scenario eval 쪽에서 다룬다. eval/README.md 참고.
"""

from dataclasses import dataclass, field
from typing import Dict

from phil_robot.config import CLASSIFIER_MODEL, PLANNER_MODEL
from phil_robot.pipeline.brain_pipeline import (
    build_direct_answer_plan,
    build_prefilter_plan,
    classify_step,
    planner_step,
)
from phil_robot.pipeline.planner import select_planner_domain
from phil_robot.pipeline.state_adapter import adapt_robot_state
from phil_robot.pipeline.validator import ValidatedPlan, build_validated_plan


@dataclass
class BrainTurnResult:
    classifier_input: str
    classifier_output: Dict
    planner_domain: str
    planner_input: str
    classifier_raw_response_text: str
    planner_raw_response_text: str
    planner_output: Dict
    robot_state: Dict
    validated_plan: ValidatedPlan = field(default_factory=ValidatedPlan)
    classifier_duration_sec: float = 0.0
    planner_duration_sec: float = 0.0
    llm_duration_sec: float = 0.0
    classifier_metrics: Dict = field(default_factory=dict)
    planner_metrics: Dict = field(default_factory=dict)


def run_brain_turn(
    user_text,
    robot_state,
    classifier_model_name=CLASSIFIER_MODEL,
    planner_model_name=PLANNER_MODEL,
    capture_metrics: bool = False,
    session=None,
):
    """
    한 턴의 LLM 처리 파이프라인을 한 번에 실행해 BrainTurnResult 로 반환한다.
    런타임 graph 가 아니라 eval/benchmark 에서 진단 데이터를 모으기 위한 경로다.

    런타임 graph 와 동일한 step 함수(build_prefilter_plan / classify_step /
    build_direct_answer_plan / planner_step)를 호출하므로 결과가 일치한다.
    """
    robot_state = adapt_robot_state(robot_state)

    # ── prefilter (raw user_text) ─────────────────────────────────────────
    prefilter = build_prefilter_plan(user_text)
    if prefilter is not None:
        classifier_output, planner_output, planner_domain = prefilter
        validated_plan = build_validated_plan(
            user_text=user_text,
            robot_state=robot_state,
            classifier_output=classifier_output,
            planner_output=planner_output,
        )
        return BrainTurnResult(
            classifier_input="",
            classifier_output=classifier_output,
            planner_domain=planner_domain,
            planner_input="",
            classifier_raw_response_text="",
            planner_raw_response_text="",
            planner_output=planner_output,
            robot_state=robot_state,
            validated_plan=validated_plan,
        )

    # ── classifier ────────────────────────────────────────────────────────
    # (clarification 텍스트 합치기는 폐기됨 — cross-turn 이어가기는 런타임 recovery 책임.
    #  eval probe 는 single-turn 진단이라 그대로 user_text 를 분류한다.)
    classifier_output, cdiag = classify_step(user_text, classifier_model_name, capture_metrics)
    planner_domain = select_planner_domain(classifier_output)

    # ── direct answer shortcut ────────────────────────────────────────────
    direct_plan = build_direct_answer_plan(user_text, classifier_output, robot_state)
    if direct_plan is not None:
        validated_plan = build_validated_plan(
            user_text=user_text,
            robot_state=robot_state,
            classifier_output=classifier_output,
            planner_output=direct_plan,
        )
        return BrainTurnResult(
            classifier_input=cdiag["classifier_input"],
            classifier_output=classifier_output,
            planner_domain=planner_domain,
            planner_input="",
            classifier_raw_response_text=cdiag["raw_response_text"],
            planner_raw_response_text="",
            planner_output=direct_plan,
            robot_state=robot_state,
            validated_plan=validated_plan,
            classifier_duration_sec=cdiag["duration_sec"],
            planner_duration_sec=0.0,
            llm_duration_sec=cdiag["duration_sec"],
            classifier_metrics=cdiag["metrics"],
            planner_metrics={},
        )

    # ── planner ───────────────────────────────────────────────────────────
    planner_output, pdiag = planner_step(
        robot_state, user_text, classifier_output, planner_domain,
        session, planner_model_name, capture_metrics,
    )
    validated_plan = build_validated_plan(
        user_text=user_text,
        robot_state=robot_state,
        classifier_output=classifier_output,
        planner_output=planner_output,
    )

    return BrainTurnResult(
        classifier_input=cdiag["classifier_input"],
        classifier_output=classifier_output,
        planner_domain=planner_domain,
        planner_input=pdiag["planner_input"],
        classifier_raw_response_text=cdiag["raw_response_text"],
        planner_raw_response_text=pdiag["raw_response_text"],
        planner_output=planner_output,
        robot_state=robot_state,
        validated_plan=validated_plan,
        classifier_duration_sec=cdiag["duration_sec"],
        planner_duration_sec=pdiag["duration_sec"],
        llm_duration_sec=cdiag["duration_sec"] + pdiag["duration_sec"],
        classifier_metrics=cdiag["metrics"],
        planner_metrics=pdiag["metrics"],
    )
