import argparse
import json
import os
import re
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple


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
    "scenario": os.path.join(CURRENT_DIR, "cases_scenario_eval.json"),
    "scenario_eval": os.path.join(CURRENT_DIR, "cases_scenario_eval.json"),
}
DEFAULT_REPORT_DIR = os.path.join(CURRENT_DIR, "reports")
DEFAULT_DOC_DIR = os.path.join(CURRENT_DIR, "eval_docs")
CHECK_LABELS = {
    "classifier.intent": "의도",
    "classifier.needs_motion": "움직임 필요",
    "classifier.needs_dialogue": "대화 필요",
    "planner.domain": "도메인",
    "planner.skills_exact": "스킬 일치",
    "planner.skills_any_of": "스킬 후보",
    "validator.valid_op_cmds_exact": "명령 일치",
    "validator.valid_op_cmds_any_of": "명령 후보",
    "validator.valid_op_cmds_contains_all": "명령 포함",
    "validator.valid_op_cmds_contains_prefixes": "명령 접두",
    "e2e.speech_contains_any": "발화 포함",
    "e2e.speech_contains_all": "발화 전체 포함",
}
FAIL_LABELS = {
    "classifier.intent": "의도 불일치",
    "classifier.needs_motion": "움직임 필요 불일치",
    "classifier.needs_dialogue": "대화 필요 불일치",
    "planner.domain": "도메인 불일치",
    "planner.skills_exact": "스킬 불일치",
    "planner.skills_any_of": "스킬 후보 불일치",
    "validator.valid_op_cmds_exact": "명령 불일치",
    "validator.valid_op_cmds_any_of": "명령 후보 불일치",
    "validator.valid_op_cmds_contains_all": "명령 누락",
    "validator.valid_op_cmds_contains_prefixes": "명령 종류 불일치",
    "e2e.speech_contains_any": "발화 표현 누락",
    "e2e.speech_contains_all": "발화 표현 누락",
}
LAYER_LABELS = {
    "classifier": "의도 분류",
    "planner": "계획 선택",
    "validator": "명령 검사",
    "e2e": "최종 발화",
}


def load_cases(cases_path: str) -> List[Dict[str, Any]]:
    with open(cases_path, "r", encoding="utf-8") as file:
        cases_data = json.load(file)

    if not isinstance(cases_data, list):
        raise ValueError("Case file must be a JSON array.")

    return cases_data


def load_json(path_name: str) -> Dict[str, Any]:
    with open(path_name, "r", encoding="utf-8") as file:
        return json.load(file)


def infer_suite_name(suite_name: str, cases_path: str) -> str:
    """suite 이름이 없으면 케이스 파일명에서 대표 이름을 추론한다."""
    if suite_name:
        return suite_name

    stem = os.path.splitext(os.path.basename(cases_path))[0]
    if stem.startswith("cases_"):
        return stem[len("cases_") :]
    return stem


def mean_num(num_list: List[float]) -> float:
    if not num_list:
        return None
    return sum(num_list) / float(len(num_list))


def median_num(num_list: List[float]) -> float:
    if not num_list:
        return None

    sorted_list = sorted(num_list)
    size_num = len(sorted_list)
    mid_num = size_num // 2
    if size_num % 2 == 1:
        return sorted_list[mid_num]
    return (sorted_list[mid_num - 1] + sorted_list[mid_num]) / 2.0


def p95_num(num_list: List[float]) -> float:
    if not num_list:
        return None

    sorted_list = sorted(num_list)
    idx_num = int(len(sorted_list) * 0.95)
    if idx_num < 0:
        idx_num = 0
    if idx_num >= len(sorted_list):
        idx_num = len(sorted_list) - 1
    return sorted_list[idx_num]


def summarize_time_rows(results: List[Dict[str, Any]], key_name: str) -> Dict[str, Any]:
    time_list: List[float] = []
    slow_row: Dict[str, Any] = None

    for result in results:
        duration_obj = result.get("durations_sec", {})
        time_val = duration_obj.get(key_name)
        if not isinstance(time_val, (int, float)):
            continue

        time_num = float(time_val)
        time_list.append(time_num)
        if slow_row is None or time_num > slow_row["time_sec"]:
            slow_row = {
                "id": result.get("id", ""),
                "time_sec": time_num,
            }

    return {
        "measured_cases": len(time_list),
        "avg_sec": mean_num(time_list),
        "median_sec": median_num(time_list),
        "p95_sec": p95_num(time_list),
        "min_sec": min(time_list) if time_list else None,
        "max_sec": max(time_list) if time_list else None,
        "slowest_case": slow_row["id"] if slow_row else "",
    }


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


def build_doc_path(path_name: str) -> str:
    abs_path = os.path.abspath(path_name)
    rel_path = os.path.relpath(abs_path, CURRENT_DIR)
    if rel_path != ".." and not rel_path.startswith(f"..{os.sep}"):
        stem, _ = os.path.splitext(rel_path)
        return os.path.join(DEFAULT_DOC_DIR, f"{stem}.md")

    stem, _ = os.path.splitext(abs_path)
    return f"{stem}.md"


def save_text(path_name: str, text: str) -> None:
    os.makedirs(os.path.dirname(path_name), exist_ok=True)
    with open(path_name, "w", encoding="utf-8") as file:
        file.write(text)


def fmt_ratio(ok_num: int, total_num: int) -> str:
    if total_num <= 0:
        return f"{ok_num}/{total_num} (기록 없음)"
    rate_num = (float(ok_num) / float(total_num)) * 100.0
    return f"{ok_num}/{total_num} ({rate_num:.1f}%)"


def fmt_sec(sec_val: Optional[float]) -> str:
    if sec_val is None:
        return "기록 없음"
    return f"{float(sec_val):.3f} s"


def norm_text(raw_val: Any) -> str:
    if raw_val is None:
        return ""
    return re.sub(r"\s+", " ", str(raw_val)).strip()


def md_cell(raw_val: Any) -> str:
    if raw_val is None:
        text = "기록 없음"
    elif isinstance(raw_val, list):
        text = ", ".join(norm_text(item) for item in raw_val if norm_text(item)) or "없음"
    elif isinstance(raw_val, bool):
        text = "true" if raw_val else "false"
    else:
        text = norm_text(raw_val) or "기록 없음"

    return text.replace("|", "\\|").replace("\n", "<br>")


def brief(raw_val: Any, limit_num: int = 120) -> str:
    text = md_cell(raw_val)
    if len(text) <= limit_num:
        return text
    return f"{text[: limit_num - 1]}…"


def label_for_check(name: str) -> str:
    return CHECK_LABELS.get(name, name)


def label_for_fail(name: str) -> str:
    return FAIL_LABELS.get(name, label_for_check(name))


def label_for_layer(name: str) -> str:
    return LAYER_LABELS.get(name, name)


def fail_labels(fail_list: List[Dict[str, Any]]) -> List[str]:
    label_list: List[str] = []
    for fail_obj in fail_list:
        name = fail_obj.get("name", "")
        if name:
            label_list.append(label_for_fail(name))
    return label_list


def fail_text(fail_list: List[Dict[str, Any]], key_name: str, limit_num: Optional[int] = None) -> str:
    if not fail_list:
        return "-"

    line_list: List[str] = []
    for fail_obj in fail_list:
        label_text = label_for_fail(fail_obj.get("name", ""))
        if limit_num is None:
            value_text = md_cell(fail_obj.get(key_name))
        else:
            value_text = brief(fail_obj.get(key_name), limit_num)
        line_list.append(f"{label_text}: {value_text}")

    return "<br>".join(line_list) if line_list else "-"


def fix_note(row_obj: Dict[str, Any]) -> str:
    fail_list = row_obj.get("failed_checks", [])
    if not fail_list:
        return "실패 원인 기록 없음"

    first_fail = fail_list[0]
    first_name = first_fail.get("name", "")
    actual_speech = norm_text(row_obj.get("actual", {}).get("speech"))
    expected_value = first_fail.get("expected", [])
    if isinstance(expected_value, list):
        expected_list = [norm_text(item) for item in expected_value]
    else:
        expected_list = [norm_text(expected_value)]

    if first_name == "validator.valid_op_cmds_exact":
        return "최종 명령이 기대 목록과 정확히 같지 않습니다."
    if first_name == "validator.valid_op_cmds_any_of":
        return "허용한 명령 조합 중 어느 것도 맞지 않았습니다."
    if first_name == "validator.valid_op_cmds_contains_all":
        return "기대한 명령이 최종 명령 목록에 다 들어가지 않았습니다."
    if first_name == "validator.valid_op_cmds_contains_prefixes":
        return "기대한 명령 종류가 접두사 기준으로 보이지 않습니다."
    if first_name == "e2e.speech_contains_any":
        if (
            any(token in {"한계", "범위"} for token in expected_list)
            and actual_speech == "지금은 해당 동작을 수행할 수 없습니다."
        ):
            return "validator에서 절대각 한계 초과를 일반 차단 문구로 덮고 있습니다. 범위/한계를 설명하는 recovery 발화를 추가하거나, generic 차단 케이스를 별도로 분리해야 합니다."
        return "기대 발화와 실제 발화가 다릅니다. validator recovery 문구를 보강하거나 케이스 기대 표현을 더 세분화해야 합니다."
    if first_name == "e2e.speech_contains_all":
        return "기대한 안내 문구 구성이 실제 최종 발화에서 깨졌습니다. validator recovery 문구를 고정하거나 케이스를 더 세분화해야 합니다."
    if first_name == "planner.domain":
        return "planner 도메인이 기대와 다릅니다."
    if first_name.startswith("planner."):
        return "planner 선택 결과가 기대와 다릅니다."
    if first_name == "classifier.intent":
        return "의도 분류가 기대와 다릅니다."
    if first_name == "classifier.needs_motion":
        return "움직임 필요 여부가 기대와 다릅니다."
    if first_name == "classifier.needs_dialogue":
        return "대화 필요 여부가 기대와 다릅니다."
    if first_name.startswith("classifier."):
        return "classifier 결과가 기대와 다릅니다."
    return "자동 비교 기준을 통과하지 못했습니다."


def latency_rows(sum_obj: Dict[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    lat_obj = sum_obj.get("latency_summary", {})
    row_list: List[Tuple[str, Dict[str, Any]]] = []
    for key_name, label_text in [
        ("classifier", "classifier"),
        ("planner", "planner"),
        ("total", "total"),
    ]:
        stage_obj = lat_obj.get(key_name)
        if isinstance(stage_obj, dict) and stage_obj.get("measured_cases"):
            row_list.append((label_text, stage_obj))
    return row_list


def build_report_md(report_path: str, report_obj: Dict[str, Any]) -> str:
    meta_obj = report_obj.get("metadata", {})
    sum_obj = report_obj.get("summary", {})
    row_list = report_obj.get("results", [])

    total_num = int(sum_obj.get("total_cases", len(row_list)))
    pass_num = int(sum_obj.get("passed_cases", sum(1 for row in row_list if row.get("passed"))))
    fail_num = int(sum_obj.get("failed_cases", total_num - pass_num))

    pass_ids = [row.get("id", "") for row in row_list if row.get("passed")]
    fail_rows = [row for row in row_list if not row.get("passed")]
    fail_ids = [row.get("id", "") for row in fail_rows]

    doc_path = build_doc_path(report_path)
    src_path = os.path.abspath(report_path)
    try:
        src_text = os.path.relpath(src_path, CURRENT_DIR)
    except ValueError:
        src_text = src_path

    line_list: List[str] = [
        f"# {os.path.basename(report_path)} 해설 리포트",
        "",
        f"- source_json: `{src_text}`",
        "",
        "## 한눈에 보기",
        "",
        "| 항목 | 내용 |",
        "| --- | --- |",
        "| 문서 종류 | 실행 결과 리포트 |",
        f"| generated_at | {md_cell(meta_obj.get('generated_at'))} |",
        f"| suite | {md_cell(meta_obj.get('suite'))} |",
        f"| cases_path | {md_cell(meta_obj.get('cases_path'))} |",
        f"| classifier_model | {md_cell(meta_obj.get('classifier_model'))} |",
        f"| planner_model | {md_cell(meta_obj.get('planner_model'))} |",
        f"| 전체 결과 | {fmt_ratio(pass_num, total_num)} |",
    ]

    for label_text, stage_obj in latency_rows(sum_obj):
        line_list.append(
            f"| {label_text} latency | avg {fmt_sec(stage_obj.get('avg_sec'))}, "
            f"median {fmt_sec(stage_obj.get('median_sec'))}, "
            f"p95 {fmt_sec(stage_obj.get('p95_sec'))} |"
        )

    line_list.extend(
        [
            "",
            "## 왜 이 실험을 했는가",
            "",
            f"`{meta_obj.get('suite', 'unknown')}` 케이스 묶음을 현재 classifier/planner 모델 조합으로 실제 평가 경로에 태웠을 때, 몇 개를 맞췄고 어디서 틀렸는지 바로 읽기 위한 실행 결과입니다.",
            "",
            "## 이번에 바꿔 보거나 고정한 점",
            "",
            f"- classifier 모델은 `{meta_obj.get('classifier_model', '기록 없음')}`로 고정했습니다.",
            f"- planner 모델은 `{meta_obj.get('planner_model', '기록 없음')}`로 고정했습니다.",
            f"- 케이스 입력은 `{meta_obj.get('cases_path', '기록 없음')}`를 그대로 사용했습니다.",
            f"- `capture_llm_metrics`는 `{md_cell(meta_obj.get('capture_llm_metrics', False))}` 상태로 실행했습니다.",
            "",
            "## 테스트 구성",
            "",
            "| 항목 | 내용 |",
            "| --- | --- |",
            f"| 전체 케이스 수 | {total_num} |",
            f"| JSON 리포트 | {md_cell(src_text)} |",
            f"| Markdown 리포트 | {md_cell(os.path.relpath(doc_path, CURRENT_DIR) if doc_path.startswith(CURRENT_DIR) else doc_path)} |",
            f"| 실패 시 종료 코드 | {1 if fail_num > 0 else 0} |",
            "",
            "## 결과 요약",
            "",
            "| 항목 | 내용 |",
            "| --- | --- |",
            f"| 전체 케이스 수 | {total_num} |",
            f"| 통과 수 | {pass_num} |",
            f"| 실패 수 | {fail_num} |",
            f"| pass rate | {fmt_ratio(pass_num, total_num)} |",
            f"| 통과한 케이스 | {md_cell(pass_ids)} |",
            f"| 실패한 케이스 | {md_cell(fail_ids)} |",
            "",
            "### 레이어별 통과율",
            "",
            "| 단계 | 통과율 |",
            "| --- | --- |",
        ]
    )

    for layer_name, layer_obj in sum_obj.get("layer_summary", {}).items():
        line_list.append(
            f"| {md_cell(label_for_layer(layer_name))} | {fmt_ratio(int(layer_obj.get('passed', 0)), int(layer_obj.get('total', 0)))} |"
        )

    line_list.extend(
        [
            "",
            "### 지연 시간 요약",
            "",
            "| 단계 | 평균 | 중앙값 | p95 | 가장 느린 케이스 |",
            "| --- | --- | --- | --- | --- |",
        ]
    )

    for label_text, stage_obj in latency_rows(sum_obj):
        line_list.append(
            f"| {label_text} | {fmt_sec(stage_obj.get('avg_sec'))} | {fmt_sec(stage_obj.get('median_sec'))} | "
            f"{fmt_sec(stage_obj.get('p95_sec'))} | {md_cell(stage_obj.get('slowest_case'))} |"
        )

    line_list.extend(["", "### 바로 고쳐야 할 항목", ""])

    if fail_rows:
        line_list.extend(
            [
                "| case id | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 최종 발화 | 바로 고칠 점 |",
                "| --- | --- | --- | --- | --- | --- |",
            ]
        )
        for row_obj in fail_rows:
            fail_list = row_obj.get("failed_checks", [])
            act_obj = row_obj.get("actual", {})
            line_list.append(
                f"| {md_cell(row_obj.get('id'))} | {md_cell(fail_labels(fail_list))} | {fail_text(fail_list, 'expected', 80)} | "
                f"{fail_text(fail_list, 'actual', 80)} | {brief(act_obj.get('speech'), 100)} | {md_cell(fix_note(row_obj))} |"
            )
    else:
        line_list.append("- 없음")

    line_list.extend(
        [
            "",
            "## 상세 표",
            "",
            "| case id | 사용자 발화 | 결과 | 실패 항목 | 기대한 것 | 실제로 나온 것 | 실제 명령 | 실제 최종 발화 | 총 시간 |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        ]
    )

    for row_obj in row_list:
        fail_list = row_obj.get("failed_checks", [])
        act_obj = row_obj.get("actual", {})
        dur_obj = row_obj.get("durations_sec", {})
        fail_name_text = "없음"
        exp_text = "-"
        act_text = "-"
        if fail_list:
            fail_name_text = md_cell(fail_labels(fail_list))
            exp_text = fail_text(fail_list, "expected")
            act_text = fail_text(fail_list, "actual")
        line_list.append(
            f"| {md_cell(row_obj.get('id'))} | {md_cell(row_obj.get('user_text'))} | "
            f"{'통과' if row_obj.get('passed') else '실패'} | {fail_name_text} | {exp_text} | {act_text} | "
            f"{md_cell(act_obj.get('valid_op_cmds'))} | {md_cell(act_obj.get('speech'))} | "
            f"{fmt_sec(dur_obj.get('total'))} |"
        )

    line_list.extend(["", "## 눈여겨볼 점", ""])

    slow_obj = sum_obj.get("latency_summary", {}).get("total", {})
    if fail_rows:
        line_list.append(
            f"- 총 {total_num}건 중 {pass_num}건 통과, {fail_num}건 실패였습니다. 실패 케이스는 `{', '.join(fail_ids)}`입니다."
        )
    else:
        line_list.append(f"- 총 {total_num}건을 모두 통과했습니다.")

    if slow_obj.get("slowest_case"):
        line_list.append(
            f"- 가장 느린 케이스는 `{slow_obj.get('slowest_case')}`였고 총 {fmt_sec(slow_obj.get('max_sec'))}가 걸렸습니다."
        )

    fallback_num = sum(1 for row in row_list if row.get("actual", {}).get("planner_is_fallback"))
    if fallback_num:
        line_list.append(f"- planner fallback 응답이 나온 케이스는 {fallback_num}건입니다.")
    else:
        line_list.append("- planner fallback 응답은 없었습니다.")

    line_list.extend(["", "## 종합 총평", ""])

    if fail_rows:
        line_list.append(
            f"이번 실행은 `{fmt_ratio(pass_num, total_num)}`로 끝났습니다. 통과한 케이스와 실패한 케이스가 명확히 갈렸으므로, 위 `바로 고쳐야 할 항목` 표의 실패 항목, 기대한 것, 실제로 나온 것을 기준으로 우선순위를 잡으면 됩니다."
        )
    else:
        line_list.append(
            f"이번 실행은 `{fmt_ratio(pass_num, total_num)}`로 전체 통과했습니다. 현재 모델 조합에서는 이 suite 기준 동작/발화 정합성이 안정적으로 유지됐습니다."
        )

    line_list.append("")
    return "\n".join(line_list)


def save_report_md(report_path: str) -> str:
    report_obj = load_json(report_path)
    md_path = build_doc_path(report_path)
    save_text(md_path, build_report_md(report_path, report_obj))
    return md_path


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


def _text_contains_all(actual: str, candidates: List[str]) -> bool:
    lowered = (actual or "").lower()
    return all(candidate.lower() in lowered for candidate in candidates)


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
    if "speech_contains_all" in expected:
        add_check(
            "e2e.speech_contains_all",
            _text_contains_all(actual["speech"], expected["speech_contains_all"]),
            expected["speech_contains_all"],
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
        "latency_summary": {
            "classifier": summarize_time_rows(results, "classifier"),
            "planner": summarize_time_rows(results, "planner"),
            "total": summarize_time_rows(results, "total"),
        },
    }


def print_results(results: List[Dict[str, Any]], summary: Dict[str, Any]) -> None:
    print("\n=== Eval Results ===")
    for result in results:
        status = "PASS" if result["passed"] else "FAIL"
        print(f"[{status}] {result['id']} :: {result['user_text']}")
        if result["failed_checks"]:
            for failed_check in result["failed_checks"]:
                print(f"  - {label_for_fail(failed_check['name'])}")
                print(f"    기대: {failed_check['expected']}")
                print(f"    실제: {failed_check['actual']}")

    print("\n=== Summary ===")
    print(
        f"Cases: {summary['passed_cases']}/{summary['total_cases']} passed "
        f"({summary['failed_cases']} failed)"
    )
    for layer_name, layer_stats in summary["layer_summary"].items():
        print(f"{label_for_layer(layer_name)}: {layer_stats['passed']}/{layer_stats['total']} checks passed")

    latency_obj = summary.get("latency_summary", {})
    for key_name, label_text in [
        ("classifier", "classifier latency"),
        ("planner", "planner latency"),
        ("total", "total latency"),
    ]:
        stage_obj = latency_obj.get(key_name, {})
        if not stage_obj.get("measured_cases"):
            continue
        print(
            f"{label_text}: avg {stage_obj.get('avg_sec'):.3f}s, "
            f"median {stage_obj.get('median_sec'):.3f}s, "
            f"p95 {stage_obj.get('p95_sec'):.3f}s"
        )


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
    parser.add_argument(
        "--scenario",
        action="store_true",
        help="Alias for --suite scenario.",
    )
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

    suite_name_arg = args.suite
    if args.scenario:
        if suite_name_arg and suite_name_arg not in {"scenario", "scenario_eval"}:
            parser.error("Use either --scenario or --suite smoke, not both.")
        suite_name_arg = "scenario"

    if not suite_name_arg and not args.cases:
        parser.error("Either --suite or --cases is required.")
    if args.report and args.save_report:
        parser.error("Use either --report or --save-report, not both.")

    cases_path = args.cases or DEFAULT_CASES[suite_name_arg]
    suite_name = infer_suite_name(suite_name_arg, cases_path)
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
        md_path = save_report_md(report_path)
        print(f"\nSaved report to: {report_path}")
        print(f"Saved markdown to: {md_path}")

    return 0 if summary["failed_cases"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
