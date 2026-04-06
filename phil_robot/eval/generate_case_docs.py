import argparse
import json
import os
from collections import Counter
from typing import Any, Dict, List


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
DOC_DIR = os.path.join(CURRENT_DIR, "eval_docs")


def load_cases(path_name: str) -> List[Dict[str, Any]]:
    with open(path_name, "r", encoding="utf-8") as file:
        data_obj = json.load(file)

    if not isinstance(data_obj, list):
        raise ValueError("Case file must be a JSON array.")

    return data_obj


def save_text(path_name: str, text: str) -> None:
    os.makedirs(os.path.dirname(path_name), exist_ok=True)
    with open(path_name, "w", encoding="utf-8") as file:
        file.write(text)


def rel_doc_path(case_path: str) -> str:
    rel_path = os.path.relpath(case_path, CURRENT_DIR)
    stem, _ = os.path.splitext(rel_path)
    return os.path.join(DOC_DIR, f"{stem}.md")


def count_tags(case_list: List[Dict[str, Any]]) -> Counter:
    tag_map: Counter = Counter()
    for case_obj in case_list:
        for tag_name in case_obj.get("tags", []):
            tag_map[str(tag_name)] += 1
    return tag_map


def count_expected(case_list: List[Dict[str, Any]], key_name: str) -> Counter:
    value_map: Counter = Counter()
    for case_obj in case_list:
        exp_obj = case_obj.get("expected", {})
        raw_val = exp_obj.get(key_name)
        if isinstance(raw_val, str) and raw_val:
            value_map[raw_val] += 1
    return value_map


def fmt_counter(value_map: Counter) -> str:
    if not value_map:
        return "기록 없음"
    row_list: List[str] = []
    for key_name, count_num in sorted(value_map.items()):
        row_list.append(f"{key_name} {count_num}건")
    return ", ".join(row_list)


def fmt_text(raw_val: Any) -> str:
    if raw_val is None:
        return "기록 없음"
    text_val = str(raw_val).strip()
    if not text_val:
        return "기록 없음"
    return text_val.replace("\n", " ").replace("|", "\\|")


def fmt_tags(tag_list: Any) -> str:
    if not isinstance(tag_list, list) or not tag_list:
        return "기록 없음"
    return ", ".join(fmt_text(tag_name) for tag_name in tag_list)


def fmt_expected(raw_val: Any) -> str:
    if raw_val is None:
        return "미채점"
    text_val = str(raw_val).strip()
    if not text_val:
        return "미채점"
    return text_val.replace("\n", " ").replace("|", "\\|")


def fmt_llm_reply(exp_obj: Dict[str, Any]) -> str:
    speech_all = exp_obj.get("speech_contains_all")
    if isinstance(speech_all, list) and speech_all:
        return "모두 포함: " + " / ".join(fmt_text(item) for item in speech_all)

    speech_list = exp_obj.get("speech_contains_any")
    if isinstance(speech_list, list) and speech_list:
        return " / ".join(fmt_text(item) for item in speech_list)
    return "미채점"


def build_case_rows(case_list: List[Dict[str, Any]]) -> List[str]:
    line_list = [
        "| case id | tags | 사용자 발화 | 기대 intent | 기대 domain | LLM 응답 |",
        "| --- | --- | --- | --- | --- | --- |",
    ]

    for case_obj in case_list:
        exp_obj = case_obj.get("expected", {})
        line_list.append(
            "| "
            f"{fmt_text(case_obj.get('id'))} | "
            f"{fmt_tags(case_obj.get('tags'))} | "
            f"{fmt_text(case_obj.get('user_text'))} | "
            f"{fmt_expected(exp_obj.get('intent'))} | "
            f"{fmt_expected(exp_obj.get('planner_domain'))} | "
            f"{fmt_llm_reply(exp_obj)} |"
        )

    return line_list


def build_md(case_path: str, case_list: List[Dict[str, Any]]) -> str:
    file_name = os.path.basename(case_path)
    source_text = os.path.relpath(case_path, CURRENT_DIR)
    tag_map = count_tags(case_list)
    intent_map = count_expected(case_list, "intent")
    domain_map = count_expected(case_list, "planner_domain")

    line_list: List[str] = [
        f"# {file_name} 해설 리포트",
        "",
        f"- source_json: `{source_text}`",
        "",
        "## 한눈에 보기",
        "",
        "| 항목 | 내용 |",
        "| --- | --- |",
        "| 문서 종류 | 입력 케이스 설명 |",
        f"| 케이스 수 | {len(case_list)} |",
        f"| 태그 종류 수 | {len(tag_map)} |",
        f"| 기대 intent 종류 | {fmt_counter(intent_map)} |",
        f"| 기대 planner_domain 종류 | {fmt_counter(domain_map)} |",
        "",
        "## 왜 이 실험을 했는가",
        "",
        "이 문서는 실행 결과표가 아니라 입력 케이스 정의를 빠르게 읽기 위한 기준표입니다.",
        "표에서 사용자 발화와 기대 `LLM 응답`을 바로 비교할 수 있게 두어, 어떤 답변을 기대하는지 먼저 파악하도록 구성했습니다.",
        "",
        "## 이번에 바꿔 보거나 고정한 점",
        "",
        "- `cases_*.json` 내용을 기준으로 표를 직접 생성하도록 맞췄습니다.",
        "- 수동 설명문 대신 JSON에 들어 있는 intent, domain, 기대 응답 단서를 앞쪽 표에 그대로 드러냈습니다.",
        "- 입력 케이스 문서는 `LLM 응답` 열 중심으로 보고, 실행 결과 문서는 실제 최종 발화를 보는 역할로 나눴습니다.",
        "",
        "## 입력 구성",
        "",
        "| 태그 | 건수 |",
        "| --- | --- |",
    ]

    for tag_name, count_num in sorted(tag_map.items()):
        line_list.append(f"| {fmt_text(tag_name)} | {count_num} |")

    line_list.extend(
        [
            "",
            "## 결과 요약",
            "",
            "| 항목 | 내용 |",
            "| --- | --- |",
            f"| 총 케이스 수 | {len(case_list)} |",
            f"| 기대 intent 분포 | {fmt_counter(intent_map)} |",
            f"| 기대 domain 분포 | {fmt_counter(domain_map)} |",
            "| 통과/실패 수 | 이 문서는 결과 리포트가 아니라 입력 정의 문서라서 기록하지 않음 |",
            "",
            "## 상세 표",
            "",
        ]
    )
    line_list.extend(build_case_rows(case_list))
    line_list.extend(
        [
            "",
            "## 눈여겨볼 점",
            "",
            "- 이 문서의 핵심은 사용자의 말과 기대 `LLM 응답`을 바로 나란히 보는 것입니다.",
            "- 실행 성공 여부와 실제 최종 발화는 대응하는 `eval_docs/reports/*.md` 문서에서 따로 확인합니다.",
            "- 케이스를 바꾼 뒤에는 이 문서도 다시 생성해 JSON과 표가 어긋나지 않게 유지합니다.",
            "",
            "## 종합 총평",
            "",
            "이 문서는 케이스 JSON을 사람이 빠르게 훑어보기 좋게 옮겨 둔 입력 기준표입니다. 수동 해설보다 JSON 원문에 가까운 정보를 먼저 보여 주는 쪽으로 맞춰, 발화 기대값을 확인하기 쉽게 했습니다.",
            "",
        ]
    )
    return "\n".join(line_list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate eval case markdown docs from cases JSON.")
    parser.add_argument(
        "--cases",
        nargs="+",
        help="Case JSON paths to convert. Default: all cases_*.json under eval/",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    case_paths = args.cases
    if not case_paths:
        case_paths = [
            os.path.join(CURRENT_DIR, file_name)
            for file_name in sorted(os.listdir(CURRENT_DIR))
            if file_name.startswith("cases_") and file_name.endswith(".json")
        ]

    for case_path in case_paths:
        case_list = load_cases(case_path)
        out_path = rel_doc_path(case_path)
        save_text(out_path, build_md(case_path, case_list))
        print(f"saved: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
