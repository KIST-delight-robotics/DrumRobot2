import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.eval.planner_json_benchmark import (  # noqa: E402
    PreparedPlannerCase,
    execute_prepared_case,
    median_num,
    p95_num,
    prepare_planner_cases,
)
from phil_robot.eval.run_eval import (  # noqa: E402
    DEFAULT_CASES,
    build_named_report_path,
    build_report_meta,
    evaluate_actual,
    load_cases,
    save_report,
)


MANIFEST_PATH = os.path.join(CURRENT_DIR, "planner_benchmark_round1_manifest.json")
ROUND_JSON = os.path.join(CURRENT_DIR, "PLANNER_MODEL_BENCHMARK_ROUND1.json")
ROUND_MD = os.path.join(CURRENT_DIR, "PLANNER_MODEL_BENCHMARK_ROUND1_KR.md")
PY_PATH = "/home/shy/miniforge3/envs/drum4/bin/python"

DOMAIN_SET = {"play", "status", "stop"}


def load_json(path_name: str) -> Dict[str, Any]:
    with open(path_name, "r", encoding="utf-8") as file:
        return json.load(file)


def save_json(path_name: str, data_obj: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path_name), exist_ok=True)
    with open(path_name, "w", encoding="utf-8") as file:
        json.dump(data_obj, file, ensure_ascii=False, indent=2)


def save_text(path_name: str, text: str) -> None:
    os.makedirs(os.path.dirname(path_name), exist_ok=True)
    with open(path_name, "w", encoding="utf-8") as file:
        file.write(text)


def read_cmd(cmd_list: List[str]) -> Dict[str, Any]:
    proc = subprocess.run(
        cmd_list,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    return {
        "ok": proc.returncode == 0,
        "code": proc.returncode,
        "out": proc.stdout.strip(),
        "err": proc.stderr.strip(),
    }


def mean_num(num_list: List[float]) -> Optional[float]:
    if not num_list:
        return None
    return sum(num_list) / float(len(num_list))


def fmt_num(num_val: Optional[float], digits: int = 2) -> str:
    if num_val is None:
        return "N/A"
    return f"{num_val:.{digits}f}"


def fmt_rate(hit_num: int, total_num: int) -> str:
    if total_num <= 0:
        return "N/A"
    return f"{(float(hit_num) / float(total_num)) * 100.0:.1f}%"


def rel_path(path_name: str) -> str:
    return os.path.relpath(path_name, CURRENT_DIR)


def read_version(cmd_list: List[str]) -> str:
    cmd_obj = read_cmd(cmd_list)
    if not cmd_obj["ok"]:
        return ""
    return cmd_obj["out"]


def list_models() -> List[Dict[str, str]]:
    cmd_obj = read_cmd(["ollama", "list"])
    if not cmd_obj["ok"]:
        raise RuntimeError(cmd_obj["err"] or "ollama list failed")

    row_list: List[Dict[str, str]] = []
    for line in cmd_obj["out"].splitlines()[1:]:
        if not line.strip():
            continue
        part_list = re.split(r"\s{2,}", line.strip(), maxsplit=3)
        if len(part_list) < 3:
            continue
        row_obj = {
            "name": part_list[0],
            "id": part_list[1],
            "size": part_list[2],
            "modified": part_list[3] if len(part_list) > 3 else "",
        }
        row_list.append(row_obj)
    return row_list


def find_model(row_list: List[Dict[str, str]], model_tag: str) -> Optional[Dict[str, str]]:
    for row_obj in row_list:
        if row_obj.get("name") == model_tag:
            return row_obj
    return None


def parse_show(show_text: str) -> Dict[str, str]:
    keep_set = {"architecture", "parameters", "context length", "embedding length", "quantization"}
    meta_obj: Dict[str, str] = {}
    for line in show_text.splitlines():
        if not line.startswith("    "):
            continue
        part_list = re.split(r"\s{2,}", line.strip(), maxsplit=1)
        if len(part_list) != 2:
            continue
        key_name = part_list[0]
        if key_name not in keep_set:
            continue
        meta_obj[key_name.replace(" ", "_")] = part_list[1]
    return meta_obj


def parse_from(mod_text: str) -> str:
    for line in mod_text.splitlines():
        line = line.strip()
        if line.startswith("# FROM "):
            return line[len("# FROM ") :].strip()
    return ""


def build_env() -> Dict[str, Any]:
    return {
        "ollama_version": read_version(["ollama", "--version"]),
        "python_path": PY_PATH,
        "python_version": read_version([PY_PATH, "-V"]),
    }


def prep_model(item: Dict[str, Any], auto_pull: bool) -> Dict[str, Any]:
    req_tag = item["requested_tag"]
    pull_tags = list(item.get("pull_tags", [])) or [req_tag]
    err_list: List[str] = []

    for try_tag in pull_tags:
        row_list = list_models()
        row_obj = find_model(row_list, try_tag)
        prep_src = "installed"

        if row_obj is None:
            if not auto_pull:
                err_list.append(f"{try_tag}: missing and auto_pull=false")
                continue
            pull_obj = read_cmd(["ollama", "pull", try_tag])
            if not pull_obj["ok"]:
                err_msg = pull_obj["err"] or pull_obj["out"] or "pull failed"
                err_list.append(f"{try_tag}: {err_msg}")
                continue
            prep_src = "pulled"
            row_list = list_models()
            row_obj = find_model(row_list, try_tag)

        show_obj = read_cmd(["ollama", "show", try_tag])
        if not show_obj["ok"]:
            err_msg = show_obj["err"] or show_obj["out"] or "show failed"
            err_list.append(f"{try_tag}: {err_msg}")
            continue

        mod_obj = read_cmd(["ollama", "show", try_tag, "--modelfile"])
        exp_tag = parse_from(mod_obj["out"]) if mod_obj["ok"] else ""
        res_tag = exp_tag if exp_tag and not exp_tag.startswith("/") else try_tag

        row_list = list_models()
        row_obj = find_model(row_list, res_tag) or find_model(row_list, try_tag)

        return {
            "status": "ready",
            "requested_tag": req_tag,
            "candidate_tag": try_tag,
            "resolved_tag": res_tag,
            "model_id": row_obj.get("id", "") if row_obj else "",
            "model_size": row_obj.get("size", "") if row_obj else "",
            "prep_source": prep_src,
            "show_meta": parse_show(show_obj["out"]),
            "modelfile_from": exp_tag,
            "errors": [],
        }

    return {
        "status": "prep_fail",
        "requested_tag": req_tag,
        "candidate_tag": "",
        "resolved_tag": "",
        "model_id": "",
        "model_size": "",
        "prep_source": "",
        "show_meta": {},
        "modelfile_from": "",
        "errors": err_list,
    }


def build_smoke_row(prepared_case: PreparedPlannerCase, planner_name: str) -> Dict[str, Any]:
    exec_obj = execute_prepared_case(
        prepared_case,
        planner_name=planner_name,
        capture_metrics=True,
    )
    actual = exec_obj["actual"]
    checks, failed_checks, passed = evaluate_actual(prepared_case.expected, actual)

    classifier_wall_sec = prepared_case.classifier_metrics.get("wall_sec")
    if not isinstance(classifier_wall_sec, (int, float)):
        classifier_wall_sec = 0.0

    planner_wall_sec = exec_obj["planner_metrics"].get("wall_sec")
    if not isinstance(planner_wall_sec, (int, float)):
        planner_wall_sec = exec_obj["planner_duration_sec"]

    return {
        "id": prepared_case.id,
        "tags": list(prepared_case.tags),
        "user_text": prepared_case.user_text,
        "passed": passed,
        "checks": checks,
        "failed_checks": failed_checks,
        "actual": actual,
        "durations_sec": {
            "classifier": classifier_wall_sec,
            "planner": planner_wall_sec,
            "total": classifier_wall_sec + planner_wall_sec,
        },
        "llm_metrics": {
            "classifier": dict(prepared_case.classifier_metrics),
            "planner": dict(exec_obj["planner_metrics"]),
        },
        "benchmark_fixture": {
            "classifier_result": dict(prepared_case.classifier_result),
            "planner_domain": prepared_case.planner_domain,
            "planner_input_json": prepared_case.planner_input_json,
            "planner_input_chars": exec_obj["planner_input_chars"],
            "planner_response_chars": exec_obj["planner_response_chars"],
            "planner_enabled": prepared_case.planner_enabled,
            "shortcut_reason": prepared_case.shortcut_reason,
        },
    }


def summarize_quality_rows(row_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    skill_total = 0
    skill_pass = 0
    cmd_total = 0
    cmd_pass = 0
    speech_total = 0
    speech_pass = 0

    for row_obj in row_list:
        for check_obj in row_obj["checks"]:
            check_name = check_obj["name"]
            if check_name.startswith("planner.skills_"):
                skill_total += 1
                if check_obj["passed"]:
                    skill_pass += 1
            if check_name.startswith("validator."):
                cmd_total += 1
                if check_obj["passed"]:
                    cmd_pass += 1
            if check_name == "e2e.speech_contains_any":
                speech_total += 1
                if check_obj["passed"]:
                    speech_pass += 1

    return {
        "skill_selection": {
            "passed": skill_pass,
            "total": skill_total,
            "pass_rate": (float(skill_pass) / float(skill_total)) if skill_total else None,
        },
        "valid_command": {
            "passed": cmd_pass,
            "total": cmd_total,
            "pass_rate": (float(cmd_pass) / float(cmd_total)) if cmd_total else None,
        },
        "speech": {
            "passed": speech_pass,
            "total": speech_total,
            "pass_rate": (float(speech_pass) / float(speech_total)) if speech_total else None,
        },
    }


def summarize_smoke_latency_rows(row_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    wall_list: List[float] = []
    input_len_list: List[float] = []
    output_len_list: List[float] = []
    slow_row: Optional[Dict[str, Any]] = None

    for row_obj in row_list:
        if not row_obj["actual"].get("planner_called", True):
            continue

        plan_metrics = row_obj.get("llm_metrics", {}).get("planner", {})
        wall_sec = plan_metrics.get("wall_sec")
        if not isinstance(wall_sec, (int, float)):
            wall_sec = row_obj["durations_sec"].get("planner")
        if isinstance(wall_sec, (int, float)):
            wall_val = float(wall_sec)
            wall_list.append(wall_val)
            if slow_row is None or wall_val > slow_row["wall_sec"]:
                slow_row = {"id": row_obj["id"], "wall_sec": wall_val}

        fixture_obj = row_obj.get("benchmark_fixture", {})
        input_len_list.append(float(fixture_obj.get("planner_input_chars", 0)))
        output_len_list.append(float(fixture_obj.get("planner_response_chars", 0)))

    return {
        "measured_cases": len(wall_list),
        "avg_planner_sec": mean_num(wall_list),
        "median_planner_sec": median_num(wall_list),
        "p95_planner_sec": p95_num(wall_list),
        "min_planner_sec": min(wall_list) if wall_list else None,
        "max_planner_sec": max(wall_list) if wall_list else None,
        "slowest_case": slow_row["id"] if slow_row else "",
        "avg_input_chars": mean_num(input_len_list),
        "avg_response_chars": mean_num(output_len_list),
    }


def summarize_results(row_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    total_cases = len(row_list)
    passed_cases = sum(1 for row_obj in row_list if row_obj["passed"])

    layer_summary: Dict[str, Dict[str, int]] = {}
    for row_obj in row_list:
        for check_obj in row_obj["checks"]:
            layer_name = check_obj["name"].split(".", 1)[0]
            if layer_name not in layer_summary:
                layer_summary[layer_name] = {"passed": 0, "total": 0}
            layer_summary[layer_name]["total"] += 1
            if check_obj["passed"]:
                layer_summary[layer_name]["passed"] += 1

    return {
        "total_cases": total_cases,
        "passed_cases": passed_cases,
        "failed_cases": total_cases - passed_cases,
        "layer_summary": layer_summary,
        "quality_summary": summarize_quality_rows(row_list),
        "planner_latency": summarize_smoke_latency_rows(row_list),
    }


def run_warm(prepared_list: List[PreparedPlannerCase], planner_tag: str) -> None:
    for prepared_case in prepared_list:
        if not prepared_case.planner_enabled:
            continue
        execute_prepared_case(prepared_case, planner_name=planner_tag, capture_metrics=True)
        return


def run_smoke(
    prepared_list: List[PreparedPlannerCase],
    case_path: str,
    suite_name: str,
    clf_tag: str,
    prep_obj: Dict[str, Any],
    env_obj: Dict[str, Any],
) -> Dict[str, Any]:
    plan_tag = prep_obj["resolved_tag"]
    run_warm(prepared_list, plan_tag)

    row_list = [build_smoke_row(prepared_case, plan_tag) for prepared_case in prepared_list]
    sum_obj = summarize_results(row_list)

    rep_path = build_named_report_path("smoke", clf_tag, plan_tag)
    meta_obj = build_report_meta(
        suite_name=suite_name,
        cases_path=case_path,
        classifier_name=clf_tag,
        planner_name=plan_tag,
        extra_meta={
            "requested_planner_tag": prep_obj["requested_tag"],
            "resolved_planner_tag": prep_obj["resolved_tag"],
            "resolved_planner_id": prep_obj["model_id"],
            "ollama_version": env_obj["ollama_version"],
            "python_path": env_obj["python_path"],
            "python_version": env_obj["python_version"],
            "warmup_excluded": True,
            "json_production_path": True,
            "planner_benchmark_mode": "json_only_fixed_classifier_fixture",
            "fixed_classifier_result": True,
            "fixed_planner_input_json": True,
        },
    )
    save_report(rep_path, row_list, sum_obj, meta_obj)

    return {
        "report_path": rep_path,
        "summary": sum_obj,
        "results": row_list,
    }


def build_note(row_obj: Dict[str, Any]) -> str:
    fail_list = [check_obj["name"] for check_obj in row_obj.get("failed_checks", [])]
    return f"{row_obj['id']}: {', '.join(fail_list)}"


def judge_gate(row_list: List[Dict[str, Any]], sum_obj: Dict[str, Any]) -> Dict[str, Any]:
    safety_hits: List[str] = []
    parser_hits: List[str] = []
    domain_hits: List[str] = []
    count_map: Dict[str, int] = {}
    note_list: List[str] = []

    for row_obj in row_list:
        if row_obj["failed_checks"]:
            note_list.append(build_note(row_obj))

        if "safety" in row_obj.get("tags", []) and row_obj["actual"]["valid_op_cmds"]:
            safety_hits.append(row_obj["id"])

        if row_obj["actual"].get("planner_called") and (
            not row_obj["actual"].get("planner_parse_ok", True)
            or row_obj["actual"].get("planner_is_fallback", False)
        ):
            parser_hits.append(row_obj["id"])

        for check_obj in row_obj["failed_checks"]:
            check_name = check_obj["name"]
            if check_name == "planner.domain":
                exp_dom = check_obj.get("expected")
                if exp_dom in DOMAIN_SET:
                    domain_hits.append(row_obj["id"])
            if check_name.startswith("planner.") or check_name.startswith("validator."):
                count_map[check_name] = count_map.get(check_name, 0) + 1

    repeat_hits = [key_name for key_name, hit_count in count_map.items() if hit_count >= 2]
    fail_list: List[str] = []
    if safety_hits:
        fail_list.append("safety case command survived")
    if len(domain_hits) >= 2:
        fail_list.append("clear domain mismatch repeated")
    if len(parser_hits) >= 2:
        fail_list.append("planner parse or fallback repeated")
    if repeat_hits:
        fail_list.append("planner or validator mismatch repeated")

    if fail_list:
        status = "fail"
    elif sum_obj["failed_cases"] == 0:
        status = "pass"
    elif sum_obj["failed_cases"] == 1:
        status = "hold_review"
    else:
        status = "fail"

    review_list = note_list if status == "hold_review" else []
    return {
        "status": status,
        "fail_reasons": fail_list,
        "review_notes": review_list,
        "safety_hits": safety_hits,
        "parser_hits": parser_hits,
        "repeat_hits": repeat_hits,
    }


def build_round_md(round_obj: Dict[str, Any]) -> str:
    line_list: List[str] = []
    line_list.append("# Planner Model Benchmark Round 1")
    line_list.append("")
    line_list.append(f"- generated_at: `{round_obj['generated_at']}`")
    line_list.append(f"- cases_path: `{round_obj['cases_path']}`")
    line_list.append(f"- classifier_model: `{round_obj['classifier_model']}`")
    line_list.append(f"- ollama_version: `{round_obj['env']['ollama_version']}`")
    line_list.append(f"- python_env: `{round_obj['env']['python_path']} ({round_obj['env']['python_version']})`")
    line_list.append(f"- benchmark_mode: `{round_obj['benchmark_mode']}`")
    line_list.append(f"- stop_before_compare: `{round_obj['stop_before_compare']}`")
    line_list.append("")
    line_list.append("## Status Table")
    line_list.append("")
    line_list.append("| order | family | requested_tag | resolved_tag | model_id | prep | gate | smoke_pass | avg_dt | median_dt | p95_dt | skill_q | cmd_q | speech_q | smoke_report |")
    line_list.append("| ---: | --- | --- | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |")
    for item in round_obj["models"]:
        smoke_obj = item.get("smoke", {})
        smoke_sum = smoke_obj.get("summary", {})
        latency_obj = smoke_sum.get("planner_latency", {})
        quality_obj = smoke_sum.get("quality_summary", {})
        smoke_text = "N/A"
        if smoke_sum:
            smoke_text = f"{smoke_sum.get('passed_cases', 0)}/{smoke_sum.get('total_cases', 0)}"
        smoke_rep = rel_path(smoke_obj["report_path"]) if smoke_obj.get("report_path") else "-"
        skill_obj = quality_obj.get("skill_selection", {})
        cmd_obj = quality_obj.get("valid_command", {})
        speech_obj = quality_obj.get("speech", {})
        line_list.append(
            f"| {item['order']} | {item['family']} | {item['requested_tag']} | "
            f"{item.get('resolved_tag', '') or '-'} | {item.get('model_id', '') or '-'} | "
            f"{item['prep_status']} | {item['gate']['status']} | {smoke_text} | "
            f"{fmt_num(latency_obj.get('avg_planner_sec'))} | {fmt_num(latency_obj.get('median_planner_sec'))} | "
            f"{fmt_num(latency_obj.get('p95_planner_sec'))} | "
            f"{fmt_rate(skill_obj.get('passed', 0), skill_obj.get('total', 0))} | "
            f"{fmt_rate(cmd_obj.get('passed', 0), cmd_obj.get('total', 0))} | "
            f"{fmt_rate(speech_obj.get('passed', 0), speech_obj.get('total', 0))} | "
            f"{smoke_rep} |"
        )
    line_list.append("")
    line_list.append("## Pending Human Review")
    line_list.append("")
    note_num = 0
    for item in round_obj["models"]:
        review_list = item["gate"].get("review_notes", [])
        if not review_list:
            continue
        note_num += 1
        line_list.append(f"### {item['requested_tag']}")
        for note_text in review_list:
            line_list.append(f"- {note_text}")
        line_list.append("")
    if note_num == 0:
        line_list.append("- 없음")
        line_list.append("")
    line_list.append("## Notes")
    line_list.append("")
    line_list.append("- 이 round benchmark 는 JSON production planner path 만 사용한다.")
    line_list.append("- classifier 는 케이스당 한 번만 실행해 `classifier_result` 와 `planner_input_json` 을 고정하고, 각 planner 모델은 같은 fixture 위에서만 비교했다.")
    line_list.append("- avg / median / p95 dt 는 별도 str 모드 비교가 아니라 smoke run 중 planner wall-clock 기준이다.")
    line_list.append("- 비교/추천 문단은 의도적으로 넣지 않았다.")
    return "\n".join(line_list)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run JSON-only planner benchmark with fixed classifier fixtures.")
    parser.add_argument("--manifest", default=MANIFEST_PATH, help="Planner benchmark manifest JSON path.")
    parser.add_argument("--round-json", default=ROUND_JSON, help="Round summary JSON output path.")
    parser.add_argument("--round-md", default=ROUND_MD, help="Round summary markdown output path.")
    args = parser.parse_args()

    man_obj = load_json(args.manifest)
    suite_name = man_obj.get("suite", "smoke")
    case_path = man_obj.get("cases_path") or DEFAULT_CASES.get(suite_name, "")
    if not os.path.isabs(case_path):
        case_path = os.path.join(CURRENT_DIR, case_path)
    case_list = load_cases(case_path)

    clf_tag = man_obj["classifier_model"]
    auto_pull = bool(man_obj.get("auto_pull", True))
    env_obj = build_env()
    prepared_list = prepare_planner_cases(
        case_list,
        classifier_name=clf_tag,
        capture_classifier_metrics=True,
    )

    model_list: List[Dict[str, Any]] = []
    for item in man_obj["candidates"]:
        print(f"[prep] {item['requested_tag']}")
        prep_obj = prep_model(item, auto_pull=auto_pull)
        model_obj = {
            "order": item["order"],
            "family": item["family"],
            "requested_tag": item["requested_tag"],
            "prep_status": prep_obj["status"],
            "resolved_tag": prep_obj.get("resolved_tag", ""),
            "model_id": prep_obj.get("model_id", ""),
            "model_size": prep_obj.get("model_size", ""),
            "prep": prep_obj,
            "smoke": {},
            "gate": {"status": "prep_fail", "fail_reasons": prep_obj.get("errors", []), "review_notes": []},
        }

        if prep_obj["status"] != "ready":
            model_list.append(model_obj)
            continue

        try:
            print(f"[smoke] {prep_obj['resolved_tag']}")
            smoke_obj = run_smoke(
                prepared_list=prepared_list,
                case_path=case_path,
                suite_name=suite_name,
                clf_tag=clf_tag,
                prep_obj=prep_obj,
                env_obj=env_obj,
            )
            gate_obj = judge_gate(smoke_obj["results"], smoke_obj["summary"])
            model_obj["smoke"] = smoke_obj
            model_obj["gate"] = gate_obj
        except Exception as exc:
            model_obj["gate"] = {
                "status": "runner_fail",
                "fail_reasons": [str(exc)],
                "review_notes": [],
            }

        model_list.append(model_obj)

    round_obj = {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "cases_path": os.path.abspath(case_path),
        "classifier_model": clf_tag,
        "stop_before_compare": bool(man_obj.get("stop_before_compare", True)),
        "benchmark_mode": "json_only_fixed_classifier_fixture",
        "env": env_obj,
        "models": model_list,
    }

    save_json(args.round_json, round_obj)
    save_text(args.round_md, build_round_md(round_obj))

    print("")
    print(f"Saved round json: {args.round_json}")
    print(f"Saved round md: {args.round_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
