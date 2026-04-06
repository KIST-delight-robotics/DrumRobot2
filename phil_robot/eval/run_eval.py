import argparse
import json
import os
import re
import sys
from datetime import datetime
from typing import Any, Dict, List, Tuple


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.pipeline.brain_pipeline import run_brain_turn  # noqa: E402
from phil_robot.pipeline.failure import FALLBACK_MESSAGE  # noqa: E402
from phil_robot.config import CLASSIFIER_MODEL, PLANNER_MODEL  # noqa: E402


DEFAULT_CASES = {
    "smoke": os.path.join(CURRENT_DIR, "cases_smoke.json"),
}
DEFAULT_REPORT_DIR = os.path.join(CURRENT_DIR, "reports")


def load_cases(cases_path: str) -> List[Dict[str, Any]]:
    with open(cases_path, "r", encoding="utf-8") as file:
        cases_data = json.load(file)

    if not isinstance(cases_data, list):
        raise ValueError("Case file must be a JSON array.")

    return cases_data


def infer_suite_name(suite_name: str, cases_path: str) -> str:
    """suite 이름이 없으면 케이스 파일명에서 대표 이름을 추론한다."""
    if suite_name:
        return suite_name

    stem = os.path.splitext(os.path.basename(cases_path))[0]
    if stem.startswith("cases_"):
        return stem[len("cases_") :]
    return stem


def abbreviate_model_name(model_name: str) -> str:
    """
    리포트 파일명에 넣기 위한 모델 약어를 생성한다.

    예:
    - qwen3:4b-instruct-2507-q4_K_M -> q3-4b-q4km
    - qwen3:30b-a3b-instruct-2507-q4_K_M -> q3-30b-a3b-q4km
    """
    normalized = (model_name or "unknown").strip().lower()
    provider, _, remainder = normalized.partition(":")

    provider_alias = {
        "qwen3": "q3",
        "qwen2.5": "q25",
        "llama3.2": "l32",
        "llama3.1": "l31",
        "phi3": "p3",
        "gemma": "gm",
    }.get(provider, re.sub(r"[^a-z0-9]+", "", provider)[:6] or "model")

    size_tokens: List[str] = []
    size_match = re.search(r"(\d+(?:\.\d+)?)b(?:-([a-z]\d+b))?", remainder)
    if size_match:
        primary_size = size_match.group(1).replace(".", "")
        size_tokens.append(f"{primary_size}b")
        if size_match.group(2):
            size_tokens.append(size_match.group(2))

    quant_token = ""
    quant_match = re.search(r"(q\d(?:_[a-z0-9]+)+)", remainder)
    if quant_match:
        quant_token = quant_match.group(1).replace("_", "")

    parts = [provider_alias]
    parts.extend(size_tokens)
    if quant_token:
        parts.append(quant_token)

    return "-".join(parts)


def build_named_report_path(
    report_name: str,
    classifier_model_name: str,
    planner_model_name: str,
    report_dir: str = DEFAULT_REPORT_DIR,
) -> str:
    """
    표준 리포트 파일명을 생성한다.

    규칙:
    - <report>_report_<classifier약어>_<planner약어>_<YYYYMMDD_HHMM>.json
    - 동일 분에 같은 조합으로 다시 저장하면 _1, _2 ... 를 뒤에 붙인다.
    """
    os.makedirs(report_dir, exist_ok=True)

    classifier_alias = abbreviate_model_name(classifier_model_name)
    planner_alias = abbreviate_model_name(planner_model_name)
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M")
    stem = f"{report_name}_report_{classifier_alias}_{planner_alias}_{timestamp}"
    candidate = os.path.join(report_dir, f"{stem}.json")

    collision_index = 1
    while os.path.exists(candidate):
        candidate = os.path.join(report_dir, f"{stem}_{collision_index}.json")
        collision_index += 1

    return candidate


def build_report_path(
    suite_name: str,
    classifier_model_name: str,
    planner_model_name: str,
    report_dir: str = DEFAULT_REPORT_DIR,
) -> str:
    return build_named_report_path(
        report_name=suite_name,
        classifier_model_name=classifier_model_name,
        planner_model_name=planner_model_name,
        report_dir=report_dir,
    )


def _list_equals(actual: List[Any], expected: List[Any]) -> bool:
    return list(actual) == list(expected)


def _list_contains_all(actual: List[str], expected: List[str]) -> bool:
    return all(item in actual for item in expected)


def _list_startswith_all(actual: List[str], prefixes: List[str]) -> bool:
    for prefix in prefixes:
        if not any(item.startswith(prefix) for item in actual):
            return False
    return True


def _list_matches_any_of(actual: List[str], candidate_lists: List[List[str]]) -> bool:
    return any(_list_equals(actual, candidate) for candidate in candidate_lists)


def _text_contains_any(actual: str, candidates: List[str]) -> bool:
    lowered = (actual or "").lower()
    return any(candidate.lower() in lowered for candidate in candidates)


def evaluate_actual(expected: Dict[str, Any], actual: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], bool]:
    checks: List[Tuple[str, bool, Any, Any]] = []

    def add_check(name: str, passed: bool, expected_value: Any, actual_value: Any) -> None:
        checks.append((name, passed, expected_value, actual_value))

    if "intent" in expected:
        add_check("classifier.intent", actual["intent"] == expected["intent"], expected["intent"], actual["intent"])
    if "needs_motion" in expected:
        add_check(
            "classifier.needs_motion",
            actual["needs_motion"] == expected["needs_motion"],
            expected["needs_motion"],
            actual["needs_motion"],
        )
    if "needs_dialogue" in expected:
        add_check(
            "classifier.needs_dialogue",
            actual["needs_dialogue"] == expected["needs_dialogue"],
            expected["needs_dialogue"],
            actual["needs_dialogue"],
        )
    if "planner_domain" in expected:
        add_check(
            "planner.domain",
            actual["planner_domain"] == expected["planner_domain"],
            expected["planner_domain"],
            actual["planner_domain"],
        )
    if "skills_exact" in expected:
        add_check(
            "planner.skills_exact",
            _list_equals(actual["skills"], expected["skills_exact"]),
            expected["skills_exact"],
            actual["skills"],
        )
    if "skills_any_of" in expected:
        add_check(
            "planner.skills_any_of",
            _list_matches_any_of(actual["skills"], expected["skills_any_of"]),
            expected["skills_any_of"],
            actual["skills"],
        )
    if "valid_op_cmds_exact" in expected or "valid_commands_exact" in expected:
        expected_value = expected.get("valid_op_cmds_exact", expected.get("valid_commands_exact"))
        add_check(
            "validator.valid_op_cmds_exact",
            _list_equals(actual["valid_op_cmds"], expected_value),
            expected_value,
            actual["valid_op_cmds"],
        )
    if "valid_op_cmds_any_of" in expected or "valid_commands_any_of" in expected:
        expected_value = expected.get("valid_op_cmds_any_of", expected.get("valid_commands_any_of"))
        add_check(
            "validator.valid_op_cmds_any_of",
            _list_matches_any_of(actual["valid_op_cmds"], expected_value),
            expected_value,
            actual["valid_op_cmds"],
        )
    if "valid_op_cmds_contains_all" in expected or "valid_commands_contains_all" in expected:
        expected_value = expected.get("valid_op_cmds_contains_all", expected.get("valid_commands_contains_all"))
        add_check(
            "validator.valid_op_cmds_contains_all",
            _list_contains_all(actual["valid_op_cmds"], expected_value),
            expected_value,
            actual["valid_op_cmds"],
        )
    if "valid_op_cmds_contains_prefixes" in expected or "valid_commands_contains_prefixes" in expected:
        expected_value = expected.get(
            "valid_op_cmds_contains_prefixes",
            expected.get("valid_commands_contains_prefixes"),
        )
        add_check(
            "validator.valid_op_cmds_contains_prefixes",
            _list_startswith_all(actual["valid_op_cmds"], expected_value),
            expected_value,
            actual["valid_op_cmds"],
        )
    if "speech_contains_any" in expected:
        add_check(
            "e2e.speech_contains_any",
            _text_contains_any(actual["speech"], expected["speech_contains_any"]),
            expected["speech_contains_any"],
            actual["speech"],
        )

    passed = all(check[1] for check in checks)
    check_rows = [
        {
            "name": name,
            "passed": ok,
            "expected": expected_value,
            "actual": actual_value,
        }
        for name, ok, expected_value, actual_value in checks
    ]
    failed_checks = [
        {
            "name": name,
            "expected": expected_value,
            "actual": actual_value,
        }
        for name, ok, expected_value, actual_value in checks
        if not ok
    ]
    return check_rows, failed_checks, passed


def _loads_json_obj(raw_text: str) -> bool:
    if not isinstance(raw_text, str) or not raw_text.strip():
        return False

    try:
        json_obj = json.loads(raw_text)
    except json.JSONDecodeError:
        return False

    return isinstance(json_obj, dict)


def build_report_meta(
    suite_name: str,
    cases_path: str,
    classifier_name: str,
    planner_name: str,
    extra_meta: Dict[str, Any] = None,
) -> Dict[str, Any]:
    meta_obj = {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "suite": suite_name,
        "cases_path": os.path.abspath(cases_path),
        "classifier_model": classifier_name,
        "planner_model": planner_name,
    }
    if extra_meta:
        meta_obj.update(extra_meta)
    return meta_obj


def evaluate_case(
    case: Dict[str, Any],
    classifier_name: str = CLASSIFIER_MODEL,
    planner_name: str = PLANNER_MODEL,
    capture_metrics: bool = False,
) -> Dict[str, Any]:
    user_text = case["user_text"]
    robot_state = case["robot_state"]
    expected = case.get("expected", {})

    result = run_brain_turn(
        user_text,
        robot_state,
        classifier_model_name=classifier_name,
        planner_model_name=planner_name,
        capture_metrics=capture_metrics,
    )
    planner_called = bool(result.planner_raw_response_text)
    planner_parse_ok = True
    if planner_called:
        planner_parse_ok = _loads_json_obj(result.planner_raw_response_text)

    planner_is_fallback = False
    if planner_called:
        planner_reason = result.planner_result.get("reason", "")
        planner_speech = result.planner_result.get("speech", "")
        planner_is_fallback = (
            not planner_parse_ok
            or planner_speech == FALLBACK_MESSAGE
            or (isinstance(planner_reason, str) and planner_reason.startswith("LLM call failed:"))
        )

    actual = {
        "intent": result.classifier_result.get("intent"),
        "needs_motion": result.classifier_result.get("needs_motion"),
        "needs_dialogue": result.classifier_result.get("needs_dialogue"),
        "risk_level": result.classifier_result.get("risk_level"),
        "planner_domain": result.planner_domain,
        "skills": list(result.validated_plan.skills),
        "raw_op_cmds": list(result.validated_plan.raw_op_cmds),
        "expanded_op_cmds": list(result.validated_plan.expanded_op_cmds),
        "resolved_op_cmds": list(result.validated_plan.resolved_op_cmds),
        "valid_op_cmds": list(result.validated_plan.valid_op_cmds),
        "rejected_op_cmds": list(result.validated_plan.rejected_op_cmds),
        "speech": result.validated_plan.speech,
        "reason": result.validated_plan.reason,
        "classifier_raw_response_text": result.classifier_raw_response_text,
        "planner_raw_response_text": result.planner_raw_response_text,
        "planner_called": planner_called,
        "planner_parse_ok": planner_parse_ok,
        "planner_is_fallback": planner_is_fallback,
    }

    checks, failed_checks, passed = evaluate_actual(expected, actual)

    return {
        "id": case["id"],
        "tags": case.get("tags", []),
        "user_text": user_text,
        "passed": passed,
        "checks": checks,
        "failed_checks": failed_checks,
        "actual": actual,
        "durations_sec": {
            "classifier": result.classifier_duration_sec,
            "planner": result.planner_duration_sec,
            "total": result.llm_duration_sec,
        },
        "llm_metrics": {
            "classifier": dict(result.classifier_metrics),
            "planner": dict(result.planner_metrics),
        },
    }


def run_cases(
    cases: List[Dict[str, Any]],
    classifier_name: str = CLASSIFIER_MODEL,
    planner_name: str = PLANNER_MODEL,
    capture_metrics: bool = False,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    results = [
        evaluate_case(
            case,
            classifier_name=classifier_name,
            planner_name=planner_name,
            capture_metrics=capture_metrics,
        )
        for case in cases
    ]
    summary = summarize_results(results)
    return results, summary


def summarize_results(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    total = len(results)
    passed = sum(1 for result in results if result["passed"])

    layer_totals: Dict[str, Dict[str, int]] = {}
    for result in results:
        for check in result["checks"]:
            layer_name = check["name"].split(".", 1)[0]
            if layer_name not in layer_totals:
                layer_totals[layer_name] = {"passed": 0, "total": 0}
            layer_totals[layer_name]["total"] += 1
            if check["passed"]:
                layer_totals[layer_name]["passed"] += 1

    return {
        "total_cases": total,
        "passed_cases": passed,
        "failed_cases": total - passed,
        "layer_summary": layer_totals,
    }


def print_results(results: List[Dict[str, Any]], summary: Dict[str, Any]) -> None:
    print("\n=== Eval Results ===")
    for result in results:
        status = "PASS" if result["passed"] else "FAIL"
        print(f"[{status}] {result['id']} :: {result['user_text']}")
        if result["failed_checks"]:
            for failed_check in result["failed_checks"]:
                print(f"  - {failed_check['name']}")
                print(f"    expected: {failed_check['expected']}")
                print(f"    actual:   {failed_check['actual']}")

    print("\n=== Summary ===")
    print(
        f"Cases: {summary['passed_cases']}/{summary['total_cases']} passed "
        f"({summary['failed_cases']} failed)"
    )
    for layer_name, layer_stats in summary["layer_summary"].items():
        print(f"{layer_name}: {layer_stats['passed']}/{layer_stats['total']} checks passed")


def save_report(
    report_path: str,
    results: List[Dict[str, Any]],
    summary: Dict[str, Any],
    metadata: Dict[str, Any],
) -> None:
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as file:
        json.dump(
            {
                "metadata": metadata,
                "summary": summary,
                "results": results,
            },
            file,
            ensure_ascii=False,
            indent=2,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Phil Robot multi-layer evaluation cases.")
    parser.add_argument("--suite", choices=sorted(DEFAULT_CASES.keys()), help="Named case suite to run.")
    parser.add_argument("--cases", help="Explicit JSON case file to run.")
    parser.add_argument("--report", help="Explicit JSON report output path.")
    parser.add_argument(
        "--classifier-model",
        default=CLASSIFIER_MODEL,
        help="Override classifier model tag without editing config.py.",
    )
    parser.add_argument(
        "--planner-model",
        default=PLANNER_MODEL,
        help="Override planner model tag without editing config.py.",
    )
    parser.add_argument(
        "--capture-llm-metrics",
        action="store_true",
        help="Include Ollama timing/token metadata in per-case results.",
    )
    parser.add_argument(
        "--save-report",
        action="store_true",
        help="Generate a report filename automatically using the standard naming rule.",
    )
    args = parser.parse_args()

    if not args.suite and not args.cases:
        parser.error("Either --suite or --cases is required.")
    if args.report and args.save_report:
        parser.error("Use either --report or --save-report, not both.")

    cases_path = args.cases or DEFAULT_CASES[args.suite]
    suite_name = infer_suite_name(args.suite, cases_path)
    cases = load_cases(cases_path)

    results, summary = run_cases(
        cases,
        classifier_name=args.classifier_model,
        planner_name=args.planner_model,
        capture_metrics=args.capture_llm_metrics,
    )
    print_results(results, summary)

    report_path = args.report
    if args.save_report:
        report_path = build_report_path(
            suite_name=suite_name,
            classifier_model_name=args.classifier_model,
            planner_model_name=args.planner_model,
        )

    if report_path:
        metadata = build_report_meta(
            suite_name=suite_name,
            cases_path=cases_path,
            classifier_name=args.classifier_model,
            planner_name=args.planner_model,
            extra_meta={
                "capture_llm_metrics": args.capture_llm_metrics,
                "json_production_path": True,
            },
        )
        save_report(report_path, results, summary, metadata)
        print(f"\nSaved report to: {report_path}")

    return 0 if summary["failed_cases"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
