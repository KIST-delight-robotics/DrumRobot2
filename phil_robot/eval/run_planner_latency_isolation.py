import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.config import CLASSIFIER_MODEL, PLANNER_MODEL  # noqa: E402
from phil_robot.eval.planner_json_benchmark import (  # noqa: E402
    execute_prepared_case,
    median_num,
    p95_num,
    prepare_planner_cases,
)
from phil_robot.eval.run_eval import (  # noqa: E402
    DEFAULT_CASES,
    build_named_report_path,
    build_report_meta,
    load_cases,
    save_report,
)


def mean_num(num_list: List[float]) -> Optional[float]:
    if not num_list:
        return None
    return sum(num_list) / float(len(num_list))


def build_case_summary(run_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    wall_list = [float(run_obj["planner_wall_sec"]) for run_obj in run_list if isinstance(run_obj.get("planner_wall_sec"), (int, float))]
    resp_len_list = [float(run_obj["planner_response_chars"]) for run_obj in run_list]
    raw_set = {run_obj.get("planner_raw_response_text", "") for run_obj in run_list}
    cmd_set = {tuple(run_obj.get("valid_op_cmds", [])) for run_obj in run_list}
    speech_set = {run_obj.get("speech", "") for run_obj in run_list}

    return {
        "runs": len(run_list),
        "avg_latency_sec": mean_num(wall_list),
        "median_latency_sec": median_num(wall_list),
        "p95_latency_sec": p95_num(wall_list),
        "min_latency_sec": min(wall_list) if wall_list else None,
        "max_latency_sec": max(wall_list) if wall_list else None,
        "avg_response_chars": mean_num(resp_len_list),
        "unique_raw_response_count": len(raw_set),
        "unique_valid_command_count": len(cmd_set),
        "unique_speech_count": len(speech_set),
    }


def build_overall_summary(case_rows: List[Dict[str, Any]], cold_row: Dict[str, Any]) -> Dict[str, Any]:
    wall_list: List[float] = []
    input_len_list: List[float] = []
    resp_len_list: List[float] = []

    for case_row in case_rows:
        input_len_list.append(float(case_row["planner_input_chars"]))
        for run_obj in case_row["warm_runs"]:
            if isinstance(run_obj.get("planner_wall_sec"), (int, float)):
                wall_list.append(float(run_obj["planner_wall_sec"]))
            resp_len_list.append(float(run_obj["planner_response_chars"]))

    return {
        "measured_cases": len(case_rows),
        "measured_runs": len(wall_list),
        "avg_latency_sec": mean_num(wall_list),
        "median_latency_sec": median_num(wall_list),
        "p95_latency_sec": p95_num(wall_list),
        "avg_input_chars": mean_num(input_len_list),
        "avg_response_chars": mean_num(resp_len_list),
        "cold_run_case": cold_row.get("case_id", ""),
        "cold_run_latency_sec": cold_row.get("planner_wall_sec"),
    }


def run_case_once(prepared_case: Any, planner_name: str) -> Dict[str, Any]:
    exec_obj = execute_prepared_case(
        prepared_case,
        planner_name=planner_name,
        capture_metrics=True,
    )
    planner_metrics = exec_obj["planner_metrics"]
    planner_wall_sec = planner_metrics.get("wall_sec")
    if not isinstance(planner_wall_sec, (int, float)):
        planner_wall_sec = exec_obj["planner_duration_sec"]

    actual = exec_obj["actual"]
    return {
        "planner_wall_sec": planner_wall_sec,
        "planner_response_chars": exec_obj["planner_response_chars"],
        "planner_raw_response_text": actual.get("planner_raw_response_text", ""),
        "valid_op_cmds": list(actual.get("valid_op_cmds", [])),
        "speech": actual.get("speech", ""),
        "planner_parse_ok": actual.get("planner_parse_ok", True),
        "planner_is_fallback": actual.get("planner_is_fallback", False),
        "planner_metrics": dict(planner_metrics),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run JSON-only planner latency isolation benchmark.")
    parser.add_argument("--suite", choices=sorted(DEFAULT_CASES.keys()), help="Named case suite to run.")
    parser.add_argument("--cases", help="Explicit JSON case file to run.")
    parser.add_argument("--classifier-model", default=CLASSIFIER_MODEL, help="Fixed classifier model for fixture generation.")
    parser.add_argument("--planner-model", default=PLANNER_MODEL, help="Planner model to benchmark.")
    parser.add_argument("--warmup-runs", type=int, default=1, help="Excluded warm-up runs per case.")
    parser.add_argument("--repeats", type=int, default=3, help="Measured warm runs per case.")
    parser.add_argument("--report", help="Explicit JSON report output path.")
    parser.add_argument(
        "--save-report",
        action="store_true",
        help="Generate a standard planner_latency report path automatically.",
    )
    args = parser.parse_args()

    if not args.suite and not args.cases:
        parser.error("Either --suite or --cases is required.")
    if args.report and args.save_report:
        parser.error("Use either --report or --save-report, not both.")

    cases_path = args.cases or DEFAULT_CASES[args.suite]
    case_list = load_cases(cases_path)
    prepared_list = prepare_planner_cases(
        case_list,
        classifier_name=args.classifier_model,
        capture_classifier_metrics=True,
    )
    prepared_list = [prepared_case for prepared_case in prepared_list if prepared_case.planner_enabled]

    if not prepared_list:
        raise SystemExit("No planner-enabled cases found for latency isolation.")

    cold_case = prepared_list[0]
    cold_run = run_case_once(cold_case, args.planner_model)
    cold_row = {
        "case_id": cold_case.id,
        "planner_wall_sec": cold_run["planner_wall_sec"],
        "planner_response_chars": cold_run["planner_response_chars"],
    }

    case_rows: List[Dict[str, Any]] = []
    for prepared_case in prepared_list:
        for _ in range(max(args.warmup_runs, 0)):
            run_case_once(prepared_case, args.planner_model)

        warm_runs = [run_case_once(prepared_case, args.planner_model) for _ in range(max(args.repeats, 0))]
        case_rows.append(
            {
                "id": prepared_case.id,
                "tags": list(prepared_case.tags),
                "user_text": prepared_case.user_text,
                "planner_domain": prepared_case.planner_domain,
                "classifier_result": dict(prepared_case.classifier_result),
                "planner_input_json": prepared_case.planner_input_json,
                "planner_input_chars": len(prepared_case.planner_input_json),
                "warm_runs": warm_runs,
                "summary": build_case_summary(warm_runs),
            }
        )

    report_path = args.report
    if args.save_report:
        report_path = build_named_report_path("planner_latency", args.classifier_model, args.planner_model)

    summary = build_overall_summary(case_rows, cold_row)
    metadata = build_report_meta(
        suite_name="planner_latency_isolation",
        cases_path=cases_path,
        classifier_name=args.classifier_model,
        planner_name=args.planner_model,
        extra_meta={
            "json_production_path": True,
            "planner_benchmark_mode": "json_only_fixed_classifier_fixture",
            "fixed_classifier_result": True,
            "fixed_planner_input_json": True,
            "warmup_runs_per_case": args.warmup_runs,
            "measured_runs_per_case": args.repeats,
            "cold_condition_note": "cold run is recorded from the first planner-enabled case before per-case warm-up.",
        },
    )

    report_obj = {
        "metadata": metadata,
        "summary": summary,
        "cold_run": cold_row,
        "results": case_rows,
    }

    if report_path:
        os.makedirs(os.path.dirname(report_path), exist_ok=True)
        with open(report_path, "w", encoding="utf-8") as file:
            json.dump(report_obj, file, ensure_ascii=False, indent=2)
        print(f"Saved report to: {report_path}")

    print("")
    print(f"planner cases: {summary['measured_cases']}")
    print(f"avg latency: {summary['avg_latency_sec']}")
    print(f"median latency: {summary['median_latency_sec']}")
    print(f"p95 latency: {summary['p95_latency_sec']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
