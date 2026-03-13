import argparse
import json
import os
import sys
from dataclasses import asdict
from typing import Any, Dict, List, Tuple


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.pipeline.brain_pipeline import run_brain_turn  # noqa: E402


DEFAULT_CASES = {
    "smoke": os.path.join(CURRENT_DIR, "cases_smoke.json"),
}


def load_cases(cases_path: str) -> List[Dict[str, Any]]:
    with open(cases_path, "r", encoding="utf-8") as file:
        payload = json.load(file)

    if not isinstance(payload, list):
        raise ValueError("Case file must be a JSON array.")

    return payload


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


def evaluate_case(case: Dict[str, Any]) -> Dict[str, Any]:
    user_text = case["user_text"]
    robot_state = case["robot_state"]
    expected = case.get("expected", {})

    result = run_brain_turn(user_text, robot_state)

    actual = {
        "intent": result.classifier_result.get("intent"),
        "needs_motion": result.classifier_result.get("needs_motion"),
        "needs_dialogue": result.classifier_result.get("needs_dialogue"),
        "risk_level": result.classifier_result.get("risk_level"),
        "planner_domain": result.planner_domain,
        "skills": list(result.validated_plan.skills),
        "raw_commands": list(result.validated_plan.raw_commands),
        "expanded_commands": list(result.validated_plan.expanded_commands),
        "resolved_commands": list(result.validated_plan.resolved_commands),
        "valid_commands": list(result.validated_plan.valid_commands),
        "rejected_commands": list(result.validated_plan.rejected_commands),
        "speech": result.validated_plan.speech,
        "reason": result.validated_plan.reason,
    }

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
    if "valid_commands_exact" in expected:
        add_check(
            "validator.valid_commands_exact",
            _list_equals(actual["valid_commands"], expected["valid_commands_exact"]),
            expected["valid_commands_exact"],
            actual["valid_commands"],
        )
    if "valid_commands_any_of" in expected:
        add_check(
            "validator.valid_commands_any_of",
            _list_matches_any_of(actual["valid_commands"], expected["valid_commands_any_of"]),
            expected["valid_commands_any_of"],
            actual["valid_commands"],
        )
    if "valid_commands_contains_all" in expected:
        add_check(
            "validator.valid_commands_contains_all",
            _list_contains_all(actual["valid_commands"], expected["valid_commands_contains_all"]),
            expected["valid_commands_contains_all"],
            actual["valid_commands"],
        )
    if "valid_commands_contains_prefixes" in expected:
        add_check(
            "validator.valid_commands_contains_prefixes",
            _list_startswith_all(actual["valid_commands"], expected["valid_commands_contains_prefixes"]),
            expected["valid_commands_contains_prefixes"],
            actual["valid_commands"],
        )
    if "speech_contains_any" in expected:
        add_check(
            "e2e.speech_contains_any",
            _text_contains_any(actual["speech"], expected["speech_contains_any"]),
            expected["speech_contains_any"],
            actual["speech"],
        )

    passed = all(check[1] for check in checks)
    failed_checks = [
        {
            "name": name,
            "expected": expected_value,
            "actual": actual_value,
        }
        for name, ok, expected_value, actual_value in checks
        if not ok
    ]

    return {
        "id": case["id"],
        "tags": case.get("tags", []),
        "user_text": user_text,
        "passed": passed,
        "checks": [
            {
                "name": name,
                "passed": ok,
                "expected": expected_value,
                "actual": actual_value,
            }
            for name, ok, expected_value, actual_value in checks
        ],
        "failed_checks": failed_checks,
        "actual": actual,
        "durations_sec": {
            "classifier": result.classifier_duration_sec,
            "planner": result.planner_duration_sec,
            "total": result.llm_duration_sec,
        },
    }


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


def save_report(report_path: str, results: List[Dict[str, Any]], summary: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as file:
        json.dump(
            {
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
    parser.add_argument("--report", help="Optional JSON report output path.")
    args = parser.parse_args()

    if not args.suite and not args.cases:
        parser.error("Either --suite or --cases is required.")

    cases_path = args.cases or DEFAULT_CASES[args.suite]
    cases = load_cases(cases_path)

    results = [evaluate_case(case) for case in cases]
    summary = summarize_results(results)
    print_results(results, summary)

    if args.report:
        save_report(args.report, results, summary)
        print(f"\nSaved report to: {args.report}")

    return 0 if summary["failed_cases"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
