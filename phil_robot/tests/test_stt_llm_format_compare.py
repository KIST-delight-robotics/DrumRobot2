import json
import os
import re
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import ollama
import sounddevice as sd
import torch
import whisper


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_ROBOT_DIR = os.path.dirname(CURRENT_DIR)
PROJECT_ROOT = os.path.dirname(PHIL_ROBOT_DIR)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phil_robot.config import PLANNER_MODEL
from phil_robot.pipeline.planner import (
    DOMAIN_INSTRUCTIONS,
    PLANNER_DOMAIN_DEFAULT,
    PLANNER_RESPONSE_SCHEMA_EXAMPLE,
    SKILL_CATALOG_TEXT,
    get_planner_system_prompt,
    parse_plan_response,
    select_planner_domain,
)
from phil_robot.pipeline.state_adapter import adapt_robot_state, build_planner_state_summary, detect_joint_angle_query
from phil_robot.runtime.phil_client import get_robot_state_snapshot


SAMPLE_RATE = 16000
RECORD_SECONDS = 4
STT_MODEL = "small"
LLM_MODEL = PLANNER_MODEL

MODE_STR = "legacy_str"
MODE_JSON = "json"

PLAY_WORDS = ["연주", "곡", "노래", "드럼", "시작", "재생", "틀어", "쳐", "치자"]
STOP_WORDS = ["멈춰", "그만", "정지", "종료", "스톱", "중지", "stop"]
STATUS_WORDS = ["상태", "왜", "무슨 일", "에러", "오류", "무엇", "뭐 했", "뭐하고", "지금 뭐"]
MOTION_WORDS = ["손", "팔", "손목", "허리", "고개", "시선", "흔들", "움직", "들어", "돌려", "봐", "만세", "인사"]

LEGACY_ITEM_PATTERN = re.compile(r"\[(CMD|SAY):([^\]]*)\]")


def read_meta_value(response: Any, key: str) -> Any:
    if isinstance(response, dict):
        return response.get(key)
    if hasattr(response, key):
        return getattr(response, key)
    return None


def calc_token_rate(token_count: Any, duration_ns: Any) -> Optional[float]:
    if not isinstance(token_count, int):
        return None
    if not isinstance(duration_ns, int):
        return None
    if token_count < 0 or duration_ns <= 0:
        return None
    return float(token_count) / (float(duration_ns) / 1_000_000_000.0)


def ns_to_sec(duration_ns: Any) -> Optional[float]:
    if not isinstance(duration_ns, int):
        return None
    if duration_ns < 0:
        return None
    return float(duration_ns) / 1_000_000_000.0


def format_metric_value(value: Optional[float], unit: str = "") -> str:
    if value is None:
        return "N/A"
    if unit:
        return f"{value:.2f} {unit}"
    return f"{value:.2f}"


def extract_ollama_metrics(response: Any) -> Dict[str, Optional[float]]:
    prompt_tokens = read_meta_value(response, "prompt_eval_count")
    prompt_ns = read_meta_value(response, "prompt_eval_duration")
    eval_tokens = read_meta_value(response, "eval_count")
    eval_ns = read_meta_value(response, "eval_duration")
    prompt_sec = ns_to_sec(prompt_ns)
    eval_sec = ns_to_sec(eval_ns)

    meta_sec = None
    if prompt_sec is not None and eval_sec is not None:
        meta_sec = prompt_sec + eval_sec

    return {
        "prompt_tokens": prompt_tokens if isinstance(prompt_tokens, int) else None,
        "prompt_sec": prompt_sec,
        "prompt_tps": calc_token_rate(prompt_tokens, prompt_ns),
        "eval_tokens": eval_tokens if isinstance(eval_tokens, int) else None,
        "eval_sec": eval_sec,
        "eval_tps": calc_token_rate(eval_tokens, eval_ns),
        "meta_sec": meta_sec,
        "overhead_sec": None,
    }


def choose_stt_device() -> str:
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def build_test_state() -> Dict:
    state = adapt_robot_state(get_robot_state_snapshot())
    state["state"] = 0
    state["is_fixed"] = True
    state["is_lock_key_removed"] = True
    state["error_detail"] = "None"
    state["last_action"] = "None"
    state["current_song"] = "None"
    state["current_song_label"] = "None"
    state["progress"] = "idle"
    return state


def infer_intent(user_text: str) -> Dict:
    text = (user_text or "").strip()

    if not text:
        return {
            "intent": "unknown",
            "needs_motion": False,
            "needs_dialogue": True,
            "risk_level": "low",
        }

    if any(word in text for word in STOP_WORDS):
        return {
            "intent": "stop_request",
            "needs_motion": True,
            "needs_dialogue": True,
            "risk_level": "medium",
        }

    if any(word in text for word in PLAY_WORDS):
        return {
            "intent": "play_request",
            "needs_motion": True,
            "needs_dialogue": True,
            "risk_level": "medium",
        }

    if detect_joint_angle_query(text) is not None or any(word in text for word in STATUS_WORDS):
        return {
            "intent": "status_question",
            "needs_motion": False,
            "needs_dialogue": True,
            "risk_level": "low",
        }

    if any(word in text for word in MOTION_WORDS):
        return {
            "intent": "motion_request",
            "needs_motion": True,
            "needs_dialogue": True,
            "risk_level": "medium",
        }

    return {
        "intent": "chat",
        "needs_motion": False,
        "needs_dialogue": True,
        "risk_level": "low",
    }


def build_compare_input_json(
    robot_state: Dict,
    user_text: str,
    intent_result: Dict,
    planner_domain: str,
    response_mode: str,
) -> str:
    payload = {
        "robot_state": build_planner_state_summary(robot_state),
        "intent_result": intent_result,
        "planner_domain": planner_domain,
        "user_text": user_text,
        "response_mode": response_mode,
    }

    if response_mode == MODE_JSON:
        payload["response_schema"] = PLANNER_RESPONSE_SCHEMA_EXAMPLE
    else:
        payload["response_format_example"] = [
            "[CMD:look:0,90]",
            "[CMD:gesture:wave]",
            "[SAY:안녕하세요!]",
        ]

    return json.dumps(payload, ensure_ascii=False, indent=2)


def build_legacy_system_prompt(planner_domain: str) -> str:
    domain_text = DOMAIN_INSTRUCTIONS.get(planner_domain, DOMAIN_INSTRUCTIONS[PLANNER_DOMAIN_DEFAULT])

    return f"""{domain_text}

반드시 일반 텍스트 줄만 출력한다. JSON 객체, 설명문, 코드블록, 마크다운은 절대 출력하지 않는다.

planner 입력에는 다음 정보가 함께 들어온다.
- robot_state: 현재 로봇 상태 요약
- intent_result: 테스트용 추정 intent 결과
- planner_domain: 현재 planner 도메인
- user_text: 사용자 발화

공통 규칙:
- 당신의 이름은 필(Phil)이며, KIST에서 개발된 지능형 휴머노이드 드럼 로봇이다.
- intent_result 를 반드시 따른다.
- intent_result.needs_motion 이 false 면 CMD 줄은 출력하지 않는다.
- 안전 키 잠김, 연주 중, 에러 상태, 이동 중이면 무리하게 명령을 만들지 않는다.
- SAY 문장은 TTS 용 자연스러운 한국어 문장만 쓴다. 괄호 설명문은 금지한다.
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


def record_audio() -> np.ndarray:
    print(f"\n🎤 듣는 중... ({RECORD_SECONDS}초)")
    audio = sd.rec(
        int(RECORD_SECONDS * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
    )
    sd.wait()
    return audio.flatten()


def warm_up_stt(stt_model, use_fp16: bool) -> None:
    print("🔥 Whisper 예열 중...")
    try:
        dummy_audio = np.zeros(16000, dtype=np.float32)
        stt_model.transcribe(dummy_audio, fp16=use_fp16, language="ko")
    except Exception:
        pass


def transcribe_audio(stt_model, audio_data: np.ndarray, use_fp16: bool) -> Tuple[str, float]:
    start_time = time.time()
    result = stt_model.transcribe(
        audio_data,
        fp16=use_fp16,
        language="ko",
        initial_prompt="필, 드럼 로봇, 팔, 손목, 고개, 시선, 연주",
    )
    user_text = result["text"].strip()
    return user_text, time.time() - start_time


def call_planner_llm(system_prompt: str, input_json: str, response_mode: str) -> Tuple[str, float, Dict[str, Optional[float]]]:
    start_time = time.time()
    request = {
        "model": LLM_MODEL,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": input_json},
        ],
    }

    if response_mode == MODE_JSON:
        request["format"] = "json"

    response = ollama.chat(**request)
    raw_text = response.get("message", {}).get("content", "")
    return raw_text, time.time() - start_time, extract_ollama_metrics(response)


def parse_legacy_response(raw_text: str) -> Tuple[List[str], str]:
    commands: List[str] = []
    speech = ""

    if not isinstance(raw_text, str):
        return commands, speech

    for match in LEGACY_ITEM_PATTERN.finditer(raw_text):
        item_type = match.group(1)
        item_value = match.group(2).strip()
        if not item_value:
            continue

        if item_type == "CMD":
            commands.append(item_value)
        elif item_type == "SAY":
            speech = item_value

    return commands, speech


def print_turn_log(
    response_mode: str,
    user_text: str,
    stt_sec: float,
    llm_sec: float,
    turn_sec: float,
    llm_metrics: Dict[str, Optional[float]],
    intent_result: Dict,
    planner_domain: str,
    input_json: str,
    raw_text: str,
) -> None:
    mode_label = "0 / legacy str" if response_mode == MODE_STR else "1 / json"

    print("\n" + "=" * 60)
    print(f"📌 비교 모드: {mode_label}")
    print(f"🗣️ 사용자 발화: {user_text}")
    print(f"⏱️ STT 처리 시간: {stt_sec:.2f}초")
    print(f"⏱️ LLM 처리 시간: {llm_sec:.2f}초")
    print(f"⏱️ 총 처리 시간: {turn_sec:.2f}초")
    print(
        "⚙️ LLM 메트릭: "
        f"prompt_tokens={llm_metrics.get('prompt_tokens') if llm_metrics.get('prompt_tokens') is not None else 'N/A'}, "
        f"prompt_eval_sec={format_metric_value(llm_metrics.get('prompt_sec'), 's')}, "
        f"prompt_tps={format_metric_value(llm_metrics.get('prompt_tps'), 'tok/s')}, "
        f"eval_tokens={llm_metrics.get('eval_tokens') if llm_metrics.get('eval_tokens') is not None else 'N/A'}, "
        f"eval_sec={format_metric_value(llm_metrics.get('eval_sec'), 's')}, "
        f"eval_tps={format_metric_value(llm_metrics.get('eval_tps'), 'tok/s')}, "
        f"meta_sec={format_metric_value(llm_metrics.get('meta_sec'), 's')}, "
        f"overhead_sec={format_metric_value(llm_metrics.get('overhead_sec'), 's')}"
    )
    print(f"🧭 추정 intent: {intent_result}")
    print(f"🧭 planner_domain: {planner_domain}")
    print(f"🧾 planner 입력:\n{input_json}")
    print(f"🤖 원본 출력:\n{raw_text}")

    if response_mode == MODE_JSON:
        print(f"🧩 JSON 파싱 결과: {parse_plan_response(raw_text)}")
    else:
        legacy_cmds, legacy_speech = parse_legacy_response(raw_text)
        print(f"🧩 Legacy CMD: {legacy_cmds}")
        print(f"🧩 Legacy SAY: {legacy_speech}")


def main() -> None:
    print("========== [STT -> LLM FORMAT COMPARE] ==========")
    print(f"[LLM] Planner model: {LLM_MODEL}")

    stt_device = choose_stt_device()
    use_fp16 = stt_device == "cuda"
    print(f"[STT] Whisper 로딩 중... model={STT_MODEL}, device={stt_device}")
    stt_model = whisper.load_model(STT_MODEL, device=stt_device)
    warm_up_stt(stt_model, use_fp16)
    print("✅ 준비 완료")
    print("입력 키: 0=legacy str, 1=json, q=종료")

    while True:
        key = input("\n⌨️ [0] legacy str / [1] json / [q] 종료 >> ").strip().lower()
        if key == "q":
            print("비교 테스트를 종료합니다.")
            break
        if key not in {"0", "1"}:
            print("⚠️ 0, 1, q 중 하나를 입력해 주세요.")
            continue

        audio_data = record_audio()
        turn_start = time.time()
        user_text, stt_sec = transcribe_audio(stt_model, audio_data, use_fp16)

        if not user_text:
            print("⚠️ 음성이 비어 있어 이번 턴을 건너뜁니다.")
            continue

        response_mode = MODE_STR if key == "0" else MODE_JSON
        robot_state = build_test_state()
        intent_result = infer_intent(user_text)
        planner_domain = select_planner_domain(intent_result)
        input_json = build_compare_input_json(
            robot_state=robot_state,
            user_text=user_text,
            intent_result=intent_result,
            planner_domain=planner_domain,
            response_mode=response_mode,
        )

        if response_mode == MODE_JSON:
            system_prompt = get_planner_system_prompt(planner_domain)
        else:
            system_prompt = build_legacy_system_prompt(planner_domain)

        print("🧠 생각 중...")

        llm_metrics = {
            "prompt_tokens": None,
            "prompt_sec": None,
            "prompt_tps": None,
            "eval_tokens": None,
            "eval_sec": None,
            "eval_tps": None,
            "meta_sec": None,
            "overhead_sec": None,
        }
        try:
            raw_text, llm_sec, llm_metrics = call_planner_llm(system_prompt, input_json, response_mode)
        except Exception as exc:
            raw_text = f"[SAY:LLM 호출 실패: {exc}]" if response_mode == MODE_STR else json.dumps(
                {
                    "skills": [],
                    "op_cmd": [],
                    "speech": f"LLM 호출 실패: {exc}",
                    "reason": "compare test failure",
                },
                ensure_ascii=False,
            )
            llm_sec = 0.0

        meta_sec = llm_metrics.get("meta_sec")
        if meta_sec is not None:
            llm_metrics["overhead_sec"] = max(llm_sec - meta_sec, 0.0)

        turn_sec = time.time() - turn_start
        print_turn_log(
            response_mode=response_mode,
            user_text=user_text,
            stt_sec=stt_sec,
            llm_sec=llm_sec,
            turn_sec=turn_sec,
            llm_metrics=llm_metrics,
            intent_result=intent_result,
            planner_domain=planner_domain,
            input_json=input_json,
            raw_text=raw_text,
        )


if __name__ == "__main__":
    main()
