import argparse
import json
import os
import re
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import ollama


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.config import CLASSIFIER_MODEL, PLANNER_MODEL
from phil_robot.pipeline.intent_classifier import (
    CLASSIFIER_SYSTEM_PROMPT,
    build_classifier_input_json,
    normalize_intent_result,
    parse_intent_response,
)
from phil_robot.pipeline.planner import (
    DOMAIN_INSTRUCTIONS,
    PLANNER_DOMAIN_DEFAULT,
    PLANNER_RESPONSE_SCHEMA_EXAMPLE,
    SKILL_CATALOG_TEXT,
    get_planner_system_prompt,
    parse_plan_response,
    select_planner_domain,
)
from phil_robot.pipeline.state_adapter import adapt_robot_state, build_planner_state_summary


MODE_STR = "legacy_str"
MODE_JSON = "json"

CASE_PATH = os.path.join(CURRENT_DIR, "cases_smoke.json")
DOC_DIR = os.path.join(PHIL_ROBOT_DIR, "docs")
MD_PATH = os.path.join(DOC_DIR, "FORMAT_COMPARE_BENCHMARK_KR.md")

LEGACY_ITEM = re.compile(r"\[(CMD|SAY):([^\]]*)\]")
TEMP_OPT = {"temperature": 0}


def load_cases(case_path: str) -> List[Dict[str, Any]]:
    with open(case_path, "r", encoding="utf-8") as file:
        case_list = json.load(file)

    if not isinstance(case_list, list):
        raise ValueError("Case file must be a JSON array.")

    return case_list


def build_legacy_prompt(domain_name: str) -> str:
    domain_text = DOMAIN_INSTRUCTIONS.get(domain_name, DOMAIN_INSTRUCTIONS[PLANNER_DOMAIN_DEFAULT])

    return f"""{domain_text}

반드시 일반 텍스트 줄만 출력한다. JSON 객체, 설명문, 코드블록, 마크다운은 절대 출력하지 않는다.

planner 입력에는 다음 정보가 함께 들어온다.
- robot_state: 현재 로봇 상태 요약
- intent_result: 1차 classifier 결과
- planner_domain: 현재 planner 도메인
- user_text: 사용자 발화

공통 규칙:
- 당신의 이름은 필(Phil)이며, KIST에서 개발된 지능형 휴머노이드 드럼 로봇이다.
- intent_result 를 반드시 따른다.
- intent_result.needs_motion 이 false 면 CMD 줄은 출력하지 않는다.
- 안전 키 잠김, 연주 중, 에러 상태, 이동 중이면 무리하게 명령을 만들지 않는다.
- SAY 문장은 자연스러운 한국어 문장만 쓴다. 괄호 설명문은 금지한다.
- move 명령은 move:L_wrist,90 처럼 실제 모터 이름을 바로 쓴다.
- look 명령 형식은 look:pan,tilt 이다. pan 은 좌우 회전이고 오른쪽은 양수, 왼쪽은 음수다. tilt 는 상하 각도이며 정면은 90, 위는 70 근처, 아래는 110 근처다.
- 가능한 경우 skill 의미를 저수준 명령으로 직접 풀어서 출력한다.

사용 가능한 skill 카탈로그:
{SKILL_CATALOG_TEXT}

사용 가능한 low-level command 예시:
- r
- h
- s
- look:0,90
- look:30,90
- look:-30,90
- look:0,70
- look:0,110
- gesture:wave
- move:L_wrist,90
- wait:2
- p:TIM

출력 형식:
- 명령이 있으면 각 줄에 [CMD:<low-level-command>] 한 줄씩 출력한다.
- 사용자에게 말할 문장은 마지막 줄에 [SAY:<한국어 문장>] 한 줄로 출력한다.
- 명령이 없으면 [SAY:...] 한 줄만 출력한다.
- 빈 줄, 번호, 불릿, 추가 설명은 금지한다.
"""


def build_mode_input(
    robot_state: Dict[str, Any],
    user_text: str,
    intent_result: Dict[str, Any],
    domain_name: str,
    mode_name: str,
) -> str:
    payload = {
        "robot_state": build_planner_state_summary(robot_state),
        "intent_result": intent_result,
        "planner_domain": domain_name,
        "user_text": user_text,
        "response_mode": mode_name,
    }

    if mode_name == MODE_JSON:
        payload["response_schema"] = PLANNER_RESPONSE_SCHEMA_EXAMPLE
    else:
        payload["response_format_example"] = [
            "[CMD:look:0,90]",
            "[CMD:gesture:wave]",
            "[SAY:안녕하세요!]",
        ]

    return json.dumps(payload, ensure_ascii=False, indent=2)


def read_meta(resp: Any, key_name: str) -> Any:
    if isinstance(resp, dict):
        return resp.get(key_name)
    if hasattr(resp, key_name):
        return getattr(resp, key_name)
    return None


def ns_to_sec(time_ns: Any) -> Optional[float]:
    if not isinstance(time_ns, int):
        return None
    if time_ns < 0:
        return None
    return float(time_ns) / 1_000_000_000.0


def calc_tps(tok_count: Any, time_ns: Any) -> Optional[float]:
    time_sec = ns_to_sec(time_ns)
    if not isinstance(tok_count, int):
        return None
    if time_sec is None or time_sec <= 0:
        return None
    return float(tok_count) / time_sec


def extract_metrics(resp: Any, wall_sec: float) -> Dict[str, Any]:
    prompt_tok = read_meta(resp, "prompt_eval_count")
    prompt_ns = read_meta(resp, "prompt_eval_duration")
    eval_tok = read_meta(resp, "eval_count")
    eval_ns = read_meta(resp, "eval_duration")

    prompt_sec = ns_to_sec(prompt_ns)
    eval_sec = ns_to_sec(eval_ns)
    meta_sec = None
    if prompt_sec is not None and eval_sec is not None:
        meta_sec = prompt_sec + eval_sec

    over_sec = None
    if meta_sec is not None:
        over_sec = max(wall_sec - meta_sec, 0.0)

    return {
        "wall_sec": wall_sec,
        "prompt_tokens": prompt_tok if isinstance(prompt_tok, int) else None,
        "prompt_sec": prompt_sec,
        "prompt_tps": calc_tps(prompt_tok, prompt_ns),
        "eval_tokens": eval_tok if isinstance(eval_tok, int) else None,
        "eval_sec": eval_sec,
        "eval_tps": calc_tps(eval_tok, eval_ns),
        "meta_sec": meta_sec,
        "overhead_sec": over_sec,
    }


def call_chat(model_name: str, sys_text: str, user_text: str, use_json: bool) -> Tuple[str, Dict[str, Any]]:
    req: Dict[str, Any] = {
        "model": model_name,
        "messages": [
            {"role": "system", "content": sys_text},
            {"role": "user", "content": user_text},
        ],
        "options": TEMP_OPT,
    }
    if use_json:
        req["format"] = "json"

    start_sec = time.time()
    resp = ollama.chat(**req)
    wall_sec = time.time() - start_sec

    if isinstance(resp, dict):
        msg = resp.get("message", {})
        raw_text = msg.get("content", "")
    else:
        raw_text = getattr(getattr(resp, "message", None), "content", "")

    return raw_text, extract_metrics(resp, wall_sec)


def parse_legacy(raw_text: str) -> Dict[str, Any]:
    cmd_list: List[str] = []
    say_text = ""

    if isinstance(raw_text, str):
        for match in LEGACY_ITEM.finditer(raw_text):
            item_tag = match.group(1)
            item_text = match.group(2).strip()
            if not item_text:
                continue
            if item_tag == "CMD":
                cmd_list.append(item_text)
            if item_tag == "SAY":
                say_text = item_text

    return {
        "parse_ok": bool(cmd_list or say_text),
        "cmd_count": len(cmd_list),
        "speech": say_text,
    }


def parse_json(raw_text: str) -> Dict[str, Any]:
    parse_ok = False
    cmd_count = 0
    speech = ""

    try:
        json_obj = json.loads(raw_text)
        parse_ok = isinstance(json_obj, dict)
    except Exception:
        json_obj = None

    plan_obj = parse_plan_response(raw_text)
    if isinstance(plan_obj, dict):
        cmd_count = len(plan_obj.get("skills", [])) + len(plan_obj.get("op_cmd", []))
        speech = plan_obj.get("speech", "")

    return {
        "parse_ok": parse_ok,
        "cmd_count": cmd_count,
        "speech": speech,
    }


def run_classifier(user_text: str, robot_state: Dict[str, Any]) -> Tuple[Dict[str, Any], str]:
    clf_input = build_classifier_input_json(robot_state, user_text)
    raw_text, _ = call_chat(CLASSIFIER_MODEL, CLASSIFIER_SYSTEM_PROMPT, clf_input, True)
    intent_obj = parse_intent_response(raw_text)
    intent_obj = normalize_intent_result(intent_obj, user_text)
    domain_name = select_planner_domain(intent_obj)
    return intent_obj, domain_name


def run_mode(
    mode_name: str,
    user_text: str,
    robot_state: Dict[str, Any],
    intent_obj: Dict[str, Any],
    domain_name: str,
) -> Dict[str, Any]:
    mode_input = build_mode_input(robot_state, user_text, intent_obj, domain_name, mode_name)
    if mode_name == MODE_JSON:
        sys_text = get_planner_system_prompt(domain_name)
        raw_text, met_obj = call_chat(PLANNER_MODEL, sys_text, mode_input, True)
        parse_obj = parse_json(raw_text)
    else:
        sys_text = build_legacy_prompt(domain_name)
        raw_text, met_obj = call_chat(PLANNER_MODEL, sys_text, mode_input, False)
        parse_obj = parse_legacy(raw_text)

    met_obj["raw_text"] = raw_text
    met_obj["output_chars"] = len(raw_text) if isinstance(raw_text, str) else 0
    met_obj["parse_ok"] = parse_obj["parse_ok"]
    met_obj["cmd_count"] = parse_obj["cmd_count"]
    met_obj["speech"] = parse_obj["speech"]
    return met_obj


def avg_num(row_list: List[Dict[str, Any]], key_name: str) -> Optional[float]:
    val_list: List[float] = []
    for row in row_list:
        val = row.get(key_name)
        if isinstance(val, (int, float)):
            val_list.append(float(val))

    if not val_list:
        return None
    return sum(val_list) / float(len(val_list))


def build_mode_avg(row_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    total = len(row_list)
    ok_num = sum(1 for row in row_list if row.get("parse_ok"))

    return {
        "cases": total,
        "parse_ok_rate": (float(ok_num) / float(total)) if total else 0.0,
        "avg_wall_sec": avg_num(row_list, "wall_sec"),
        "avg_prompt_tokens": avg_num(row_list, "prompt_tokens"),
        "avg_prompt_sec": avg_num(row_list, "prompt_sec"),
        "avg_prompt_tps": avg_num(row_list, "prompt_tps"),
        "avg_eval_tokens": avg_num(row_list, "eval_tokens"),
        "avg_eval_sec": avg_num(row_list, "eval_sec"),
        "avg_eval_tps": avg_num(row_list, "eval_tps"),
        "avg_meta_sec": avg_num(row_list, "meta_sec"),
        "avg_overhead_sec": avg_num(row_list, "overhead_sec"),
        "avg_output_chars": avg_num(row_list, "output_chars"),
        "avg_cmd_count": avg_num(row_list, "cmd_count"),
    }


def fmt_num(num_val: Optional[float], digits: int = 2) -> str:
    if num_val is None:
        return "N/A"
    return f"{num_val:.{digits}f}"


def build_md(
    now_text: str,
    json_name: str,
    case_rows: List[Dict[str, Any]],
    str_avg: Dict[str, Any],
    json_avg: Dict[str, Any],
    case_path: str,
) -> str:
    line_list: List[str] = []
    line_list.append("# Format Compare Benchmark")
    line_list.append("")
    line_list.append(f"- generated_at: `{now_text}`")
    line_list.append(f"- case_source: `{case_path}`")
    line_list.append(f"- case_count: `{len(case_rows)}`")
    line_list.append(f"- classifier_model: `{CLASSIFIER_MODEL}`")
    line_list.append(f"- planner_model: `{PLANNER_MODEL}`")
    line_list.append("- method: classifier는 케이스마다 한 번만 실행하고, 같은 `intent_result`를 재사용해 planner 단계만 `legacy_str`와 `json`으로 비교했다.")
    line_list.append("- method: smoke 케이스 앞의 10개를 사용했고, 각 모드는 케이스당 1회 실행했다. 즉 모드별 총 10회다.")
    line_list.append("- method: Ollama 호출은 `temperature=0`으로 고정했다.")
    line_list.append("- method: 측정 전에 classifier와 planner 두 모드를 각각 1회씩 warm-up 하고, 그 호출은 통계에서 제외했다.")
    line_list.append(f"- raw_report: `{json_name}`")
    line_list.append("")
    line_list.append("## Average Table")
    line_list.append("")
    line_list.append("| mode | cases | parse_ok | avg_wall_sec | avg_prompt_tokens | avg_prompt_sec | avg_prompt_tps | avg_eval_tokens | avg_eval_sec | avg_eval_tps | avg_meta_sec | avg_overhead_sec | avg_output_chars | avg_cmd_count |")
    line_list.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    line_list.append(
        "| legacy_str | "
        f"{str_avg['cases']} | {fmt_num(str_avg['parse_ok_rate'] * 100.0)}% | {fmt_num(str_avg['avg_wall_sec'])} | "
        f"{fmt_num(str_avg['avg_prompt_tokens'])} | {fmt_num(str_avg['avg_prompt_sec'])} | {fmt_num(str_avg['avg_prompt_tps'])} | "
        f"{fmt_num(str_avg['avg_eval_tokens'])} | {fmt_num(str_avg['avg_eval_sec'])} | {fmt_num(str_avg['avg_eval_tps'])} | "
        f"{fmt_num(str_avg['avg_meta_sec'])} | {fmt_num(str_avg['avg_overhead_sec'])} | {fmt_num(str_avg['avg_output_chars'])} | {fmt_num(str_avg['avg_cmd_count'])} |"
    )
    line_list.append(
        "| json | "
        f"{json_avg['cases']} | {fmt_num(json_avg['parse_ok_rate'] * 100.0)}% | {fmt_num(json_avg['avg_wall_sec'])} | "
        f"{fmt_num(json_avg['avg_prompt_tokens'])} | {fmt_num(json_avg['avg_prompt_sec'])} | {fmt_num(json_avg['avg_prompt_tps'])} | "
        f"{fmt_num(json_avg['avg_eval_tokens'])} | {fmt_num(json_avg['avg_eval_sec'])} | {fmt_num(json_avg['avg_eval_tps'])} | "
        f"{fmt_num(json_avg['avg_meta_sec'])} | {fmt_num(json_avg['avg_overhead_sec'])} | {fmt_num(json_avg['avg_output_chars'])} | {fmt_num(json_avg['avg_cmd_count'])} |"
    )
    line_list.append(
        "| json - legacy_str | "
        f"{json_avg['cases'] - str_avg['cases']} | {fmt_num((json_avg['parse_ok_rate'] - str_avg['parse_ok_rate']) * 100.0)}%p | "
        f"{fmt_num((json_avg['avg_wall_sec'] or 0.0) - (str_avg['avg_wall_sec'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_prompt_tokens'] or 0.0) - (str_avg['avg_prompt_tokens'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_prompt_sec'] or 0.0) - (str_avg['avg_prompt_sec'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_prompt_tps'] or 0.0) - (str_avg['avg_prompt_tps'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_eval_tokens'] or 0.0) - (str_avg['avg_eval_tokens'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_eval_sec'] or 0.0) - (str_avg['avg_eval_sec'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_eval_tps'] or 0.0) - (str_avg['avg_eval_tps'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_meta_sec'] or 0.0) - (str_avg['avg_meta_sec'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_overhead_sec'] or 0.0) - (str_avg['avg_overhead_sec'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_output_chars'] or 0.0) - (str_avg['avg_output_chars'] or 0.0))} | "
        f"{fmt_num((json_avg['avg_cmd_count'] or 0.0) - (str_avg['avg_cmd_count'] or 0.0))} |"
    )
    line_list.append("")
    line_list.append("## Per-Case Table")
    line_list.append("")
    line_list.append("| case_id | planner_domain | str_wall_sec | json_wall_sec | str_eval_tokens | json_eval_tokens | str_eval_tps | json_eval_tps | str_parse | json_parse |")
    line_list.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for case_row in case_rows:
        str_row = case_row["legacy_str"]
        json_row = case_row["json"]
        line_list.append(
            f"| {case_row['id']} | {case_row['planner_domain']} | "
            f"{fmt_num(str_row.get('wall_sec'))} | {fmt_num(json_row.get('wall_sec'))} | "
            f"{fmt_num(str_row.get('eval_tokens'))} | {fmt_num(json_row.get('eval_tokens'))} | "
            f"{fmt_num(str_row.get('eval_tps'))} | {fmt_num(json_row.get('eval_tps'))} | "
            f"{str_row.get('parse_ok')} | {json_row.get('parse_ok')} |"
        )
    line_list.append("")
    line_list.append("## Notes")
    line_list.append("")
    line_list.append("- `prompt_*`는 system prompt + user JSON 전체를 읽는 prefill 구간이다.")
    line_list.append("- `eval_*`는 실제 출력 토큰을 생성하는 decode 구간이다.")
    line_list.append("- `wall_sec`는 Python 바깥에서 잰 전체 planner 호출 시간이고, `meta_sec`는 Ollama 내부 메타데이터 합이다.")
    line_list.append("- `overhead_sec`는 `wall_sec - meta_sec`로 계산한 Python/Ollama 바깥 오버헤드 추정치다.")
    line_list.append("- 이 문서는 STT를 포함하지 않는다. smoke 텍스트 입력만으로 planner 출력 형식 차이를 비교했다.")
    line_list.append("")
    return "\n".join(line_list)


def save_json(path_name: str, data_obj: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path_name), exist_ok=True)
    with open(path_name, "w", encoding="utf-8") as file:
        json.dump(data_obj, file, ensure_ascii=False, indent=2)


def save_text(path_name: str, text: str) -> None:
    os.makedirs(os.path.dirname(path_name), exist_ok=True)
    with open(path_name, "w", encoding="utf-8") as file:
        file.write(text)


def warm_up(case_obj: Dict[str, Any]) -> None:
    user_text = case_obj["user_text"]
    robot_state = adapt_robot_state(case_obj["robot_state"])

    print("[warm-up] classifier")
    intent_obj, domain_name = run_classifier(user_text, robot_state)
    print("[warm-up] legacy_str")
    run_mode(MODE_STR, user_text, robot_state, intent_obj, domain_name)
    print("[warm-up] json")
    run_mode(MODE_JSON, user_text, robot_state, intent_obj, domain_name)


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark legacy str vs json planner output formats.")
    parser.add_argument("--cases", default=CASE_PATH, help="Case JSON path.")
    parser.add_argument("--limit", type=int, default=10, help="Number of smoke cases to benchmark.")
    parser.add_argument("--md", default=MD_PATH, help="Markdown report path.")
    parser.add_argument("--json", dest="json_path", help="Raw JSON report path.")
    args = parser.parse_args()

    case_list = load_cases(args.cases)
    use_list = case_list[: max(args.limit, 0)]
    now_obj = datetime.now().astimezone()
    now_text = now_obj.isoformat(timespec="seconds")

    if args.json_path:
        json_path = args.json_path
    else:
        stamp = now_obj.strftime("%Y%m%d_%H%M%S")
        json_path = os.path.join(DOC_DIR, f"format_compare_benchmark_smoke_{stamp}.json")

    case_rows: List[Dict[str, Any]] = []
    str_rows: List[Dict[str, Any]] = []
    json_rows: List[Dict[str, Any]] = []

    if use_list:
        warm_up(use_list[0])

    for idx, case_obj in enumerate(use_list, start=1):
        case_id = case_obj["id"]
        user_text = case_obj["user_text"]
        robot_state = adapt_robot_state(case_obj["robot_state"])

        print(f"[{idx}/{len(use_list)}] {case_id} :: classifier")
        intent_obj, domain_name = run_classifier(user_text, robot_state)

        print(f"[{idx}/{len(use_list)}] {case_id} :: legacy_str")
        str_row = run_mode(MODE_STR, user_text, robot_state, intent_obj, domain_name)

        print(f"[{idx}/{len(use_list)}] {case_id} :: json")
        json_row = run_mode(MODE_JSON, user_text, robot_state, intent_obj, domain_name)

        case_row = {
            "id": case_id,
            "tags": case_obj.get("tags", []),
            "user_text": user_text,
            "planner_domain": domain_name,
            "intent_result": intent_obj,
            MODE_STR: str_row,
            MODE_JSON: json_row,
        }
        case_rows.append(case_row)
        str_rows.append(str_row)
        json_rows.append(json_row)

    str_avg = build_mode_avg(str_rows)
    json_avg = build_mode_avg(json_rows)
    report_obj = {
        "metadata": {
            "generated_at": now_text,
            "cases_path": os.path.abspath(args.cases),
            "case_count": len(use_list),
            "classifier_model": CLASSIFIER_MODEL,
            "planner_model": PLANNER_MODEL,
            "temperature": 0,
            "notes": [
                "Classifier ran once per case and planner compared legacy_str vs json with shared intent_result.",
                "One warm-up call was executed for classifier and both planner modes before timed runs.",
                "Prompt/output token metrics come from Ollama prompt_eval_* and eval_* metadata.",
            ],
        },
        "summary": {
            MODE_STR: str_avg,
            MODE_JSON: json_avg,
        },
        "results": case_rows,
    }

    md_text = build_md(now_text, os.path.basename(json_path), case_rows, str_avg, json_avg, os.path.abspath(args.cases))
    save_json(json_path, report_obj)
    save_text(args.md, md_text)

    print("")
    print(f"Saved markdown: {args.md}")
    print(f"Saved json: {json_path}")
    print("")
    print("=== Average ===")
    print(f"legacy_str avg_wall_sec: {fmt_num(str_avg['avg_wall_sec'])}")
    print(f"json       avg_wall_sec: {fmt_num(json_avg['avg_wall_sec'])}")
    print(f"legacy_str avg_eval_tok: {fmt_num(str_avg['avg_eval_tokens'])}")
    print(f"json       avg_eval_tok: {fmt_num(json_avg['avg_eval_tokens'])}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
