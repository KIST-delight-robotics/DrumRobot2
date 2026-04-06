import argparse
import json
import os
import re
import time
from datetime import datetime
from difflib import SequenceMatcher
from typing import Any, Dict, List

import librosa
import numpy as np
import whisper

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PHIL_DIR = os.path.dirname(CURRENT_DIR)
ROOT_DIR = os.path.dirname(PHIL_DIR)

if ROOT_DIR not in os.sys.path:
    os.sys.path.insert(0, ROOT_DIR)

from phil_robot.config import CLASSIFIER_MODEL, PLANNER_MODEL
from phil_robot.pipeline.brain_pipeline import run_brain_turn
from phil_robot.runtime.melo_engine import TTS_Engine


CASE_PATH = os.path.join(CURRENT_DIR, "cases_smoke.json")
REP_DIR = os.path.join(CURRENT_DIR, "reports")
STT_MODEL = "small"

STT_CFG_LIST = [
    {"name": "fp32_plain", "fp16": False, "prompt": ""},
    {"name": "fp16_plain", "fp16": True, "prompt": ""},
    {"name": "fp16_hint", "fp16": True, "prompt": "필, 드럼 로봇, 연주, 손, 팔, 손목, 고개, 시선"},
]

TTS_CFG_LIST = [
    {"name": "file_only", "stream": False, "min_len": 0},
    {"name": "stream_24", "stream": True, "min_len": 24},
    {"name": "stream_18", "stream": True, "min_len": 18},
    {"name": "stream_14", "stream": True, "min_len": 14},
]

EXTRA_TTS_TEXTS = [
    "말을 이해하지 못했어요. 다시 한 번 말해 주실 수 있나요?",
    "아직 안전 키가 해제되지 않아 움직일 수 없습니다. 안전 키를 먼저 확인해 주세요.",
    "지금은 연주 중이라 다른 동작을 할 수 없습니다. 잠시 후에 다시 말씀해 주세요.",
    "죄송합니다. 현재 팔의 관절 한계에 도달해서 멈췄습니다. 점검 중입니다.",
    "반숙 계란은 보통 4분에서 5분 정도 삶으면 좋아요. 시간이 길어지면 딱딱해져요.",
]


def load_case_list(case_path: str) -> List[Dict[str, Any]]:
    with open(case_path, "r", encoding="utf-8") as file_obj:
        return json.load(file_obj)


def unique_text_list(case_list: List[Dict[str, Any]]) -> List[str]:
    seen_set = set()
    text_list: List[str] = []
    for case_obj in case_list:
        text_val = str(case_obj.get("user_text", "")).strip()
        if not text_val or text_val in seen_set:
            continue
        seen_set.add(text_val)
        text_list.append(text_val)
    return text_list


def clean_text(text: str) -> str:
    text_val = str(text or "").strip().lower()
    text_val = re.sub(r"\s+", "", text_val)
    text_val = re.sub(r"[^\w가-힣]", "", text_val)
    return text_val


def text_score(ref_text: str, got_text: str) -> float:
    ref_val = clean_text(ref_text)
    got_val = clean_text(got_text)
    if not ref_val and not got_val:
        return 1.0
    return float(SequenceMatcher(None, ref_val, got_val).ratio())


def avg_num(num_list: List[float]) -> float:
    if not num_list:
        return 0.0
    return float(sum(num_list) / len(num_list))


def p95_num(num_list: List[float]) -> float:
    if not num_list:
        return 0.0
    sorted_list = sorted(num_list)
    idx_num = int(len(sorted_list) * 0.95)
    if idx_num >= len(sorted_list):
        idx_num = len(sorted_list) - 1
    return float(sorted_list[idx_num])


def warm_stt(stt_model, cfg_obj: Dict[str, Any]) -> None:
    dummy_audio = np.zeros(16000, dtype=np.float32)
    try:
        stt_model.transcribe(
            dummy_audio,
            fp16=cfg_obj["fp16"],
            language="ko",
            initial_prompt=cfg_obj["prompt"],
        )
    except Exception:
        pass


def synth_audio(tts_obj: TTS_Engine, text: str) -> np.ndarray:
    norm_text = tts_obj.preprocess(text)
    audio_data = np.asarray(tts_obj._render_audio(norm_text), dtype=np.float32)
    if tts_obj.sample_rate == 16000:
        return audio_data
    return np.asarray(librosa.resample(audio_data, orig_sr=tts_obj.sample_rate, target_sr=16000), dtype=np.float32)


def build_speech_rows(case_list: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    row_list: List[Dict[str, str]] = []
    for idx_num, case_obj in enumerate(case_list, start=1):
        print(f"[LLM] 답변 문장 준비 {idx_num}/{len(case_list)} :: {case_obj['id']}")
        brain_obj = run_brain_turn(
            user_text=case_obj["user_text"],
            raw_robot_state=case_obj["robot_state"],
            classifier_model_name=CLASSIFIER_MODEL,
            planner_model_name=PLANNER_MODEL,
            capture_metrics=False,
        )
        row_list.append(
            {
                "id": case_obj["id"],
                "user_text": case_obj["user_text"],
                "speech": brain_obj.validated_plan.speech,
            }
        )

    for idx_num, text_val in enumerate(EXTRA_TTS_TEXTS, start=1):
        row_list.append(
            {
                "id": f"probe_{idx_num}",
                "user_text": "",
                "speech": text_val,
            }
        )
    return row_list


def run_stt_bench(tts_obj: TTS_Engine, stt_model, text_list: List[str]) -> Dict[str, Any]:
    print(f"[STT] 같은 문장을 여러 설정으로 비교합니다. 문장 수={len(text_list)}")
    audio_map = {text_val: synth_audio(tts_obj, text_val) for text_val in text_list}
    row_list: List[Dict[str, Any]] = []
    sum_list: List[Dict[str, Any]] = []

    for cfg_obj in STT_CFG_LIST:
        print(f"[STT] '{cfg_obj['name']}' 설정을 돌려봅니다.")
        warm_stt(stt_model, cfg_obj)
        sec_list: List[float] = []
        score_list: List[float] = []
        exact_num = 0

        for text_val in text_list:
            audio_data = audio_map[text_val]
            start_sec = time.time()
            out_obj = stt_model.transcribe(
                audio_data,
                fp16=cfg_obj["fp16"],
                language="ko",
                initial_prompt=cfg_obj["prompt"],
            )
            run_sec = time.time() - start_sec
            got_text = str(out_obj.get("text", "")).strip()
            score_num = text_score(text_val, got_text)
            exact_hit = clean_text(text_val) == clean_text(got_text)

            sec_list.append(run_sec)
            score_list.append(score_num)
            if exact_hit:
                exact_num += 1

            row_list.append(
                {
                    "cfg": cfg_obj["name"],
                    "ref_text": text_val,
                    "got_text": got_text,
                    "latency_sec": round(run_sec, 4),
                    "score": round(score_num, 4),
                    "exact_hit": exact_hit,
                    "audio_sec": round(len(audio_data) / 16000.0, 4),
                }
            )

        sum_list.append(
            {
                "name": cfg_obj["name"],
                "avg_latency_sec": round(avg_num(sec_list), 4),
                "p95_latency_sec": round(p95_num(sec_list), 4),
                "avg_score": round(avg_num(score_list), 4),
                "exact_hits": exact_num,
                "case_count": len(text_list),
                "fp16": cfg_obj["fp16"],
                "prompt": cfg_obj["prompt"],
            }
        )

    best_score = max(item["avg_score"] for item in sum_list)
    pick_list = [item for item in sum_list if item["avg_score"] >= best_score - 0.01]
    best_obj = min(pick_list, key=lambda item: item["avg_latency_sec"])
    return {
        "summary": sum_list,
        "rows": row_list,
        "best": best_obj,
    }


def measure_tts_file(tts_obj: TTS_Engine, text: str) -> Dict[str, Any]:
    clean_val = tts_obj.preprocess(text)
    start_sec = time.time()
    audio_data = tts_obj._render_audio(clean_val)
    synth_sec = time.time() - start_sec
    return {
        "chunk_count": 1,
        "first_audio_sec": synth_sec,
        "synth_total_sec": synth_sec,
        "audio_sec": len(audio_data) / float(tts_obj.sample_rate),
        "split_hit": False,
        "avg_chunk_len": len(clean_val),
    }


def measure_tts_stream(tts_obj: TTS_Engine, text: str, min_len: int) -> Dict[str, Any]:
    clean_val = tts_obj.preprocess(text)
    old_len = tts_obj.stream_min_len
    tts_obj.stream_min_len = min_len
    try:
        chunk_list = tts_obj._split_stream_text(clean_val)
    finally:
        tts_obj.stream_min_len = old_len

    if len(chunk_list) < 2:
        row_obj = measure_tts_file(tts_obj, text)
        row_obj["fallback_file"] = True
        return row_obj

    first_sec = 0.0
    total_sec = 0.0
    total_audio = 0.0
    len_list: List[int] = []

    for idx_num, chunk_text in enumerate(chunk_list):
        start_sec = time.time()
        audio_data = tts_obj._render_audio(chunk_text)
        run_sec = time.time() - start_sec
        if idx_num == 0:
            first_sec = run_sec
        total_sec += run_sec
        total_audio += len(audio_data) / float(tts_obj.sample_rate)
        len_list.append(len(chunk_text))

    return {
        "chunk_count": len(chunk_list),
        "first_audio_sec": first_sec,
        "synth_total_sec": total_sec,
        "audio_sec": total_audio,
        "split_hit": True,
        "avg_chunk_len": avg_num([float(item) for item in len_list]),
        "fallback_file": False,
    }


def run_tts_bench(tts_obj: TTS_Engine, speech_rows: List[Dict[str, str]]) -> Dict[str, Any]:
    print(f"[TTS] 실제 답변 문장을 여러 재생 방식으로 비교합니다. 문장 수={len(speech_rows)}")
    row_list: List[Dict[str, Any]] = []
    sum_list: List[Dict[str, Any]] = []

    for cfg_obj in TTS_CFG_LIST:
        print(f"[TTS] '{cfg_obj['name']}' 설정을 돌려봅니다.")
        first_list: List[float] = []
        synth_list: List[float] = []
        chunk_list: List[float] = []
        split_num = 0

        for speech_obj in speech_rows:
            if cfg_obj["stream"]:
                met_obj = measure_tts_stream(tts_obj, speech_obj["speech"], cfg_obj["min_len"])
            else:
                met_obj = measure_tts_file(tts_obj, speech_obj["speech"])
                met_obj["fallback_file"] = False

            if met_obj["split_hit"]:
                split_num += 1

            first_list.append(float(met_obj["first_audio_sec"]))
            synth_list.append(float(met_obj["synth_total_sec"]))
            chunk_list.append(float(met_obj["chunk_count"]))

            row_obj = dict(speech_obj)
            row_obj.update(
                {
                    "cfg": cfg_obj["name"],
                    "first_audio_sec": round(float(met_obj["first_audio_sec"]), 4),
                    "synth_total_sec": round(float(met_obj["synth_total_sec"]), 4),
                    "audio_sec": round(float(met_obj["audio_sec"]), 4),
                    "chunk_count": int(met_obj["chunk_count"]),
                    "split_hit": bool(met_obj["split_hit"]),
                    "fallback_file": bool(met_obj["fallback_file"]),
                    "avg_chunk_len": round(float(met_obj["avg_chunk_len"]), 2),
                }
            )
            row_list.append(row_obj)

        sum_list.append(
            {
                "name": cfg_obj["name"],
                "avg_first_audio_sec": round(avg_num(first_list), 4),
                "p95_first_audio_sec": round(p95_num(first_list), 4),
                "avg_synth_total_sec": round(avg_num(synth_list), 4),
                "avg_chunk_count": round(avg_num(chunk_list), 4),
                "split_cases": split_num,
                "case_count": len(speech_rows),
                "stream": cfg_obj["stream"],
                "min_len": cfg_obj["min_len"],
            }
        )

    best_obj = min(
        sum_list,
        key=lambda item: (item["avg_first_audio_sec"], item["avg_chunk_count"], item["avg_synth_total_sec"]),
    )
    return {
        "summary": sum_list,
        "rows": row_list,
        "best": best_obj,
    }


def save_report(report_obj: Dict[str, Any]) -> str:
    os.makedirs(REP_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(REP_DIR, f"voice_io_benchmark_{stamp}.json")
    with open(out_path, "w", encoding="utf-8") as file_obj:
        json.dump(report_obj, file_obj, ensure_ascii=False, indent=2)
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="STT/TTS I/O benchmark runner")
    parser.add_argument("--cases", default=CASE_PATH, help="benchmark case json path")
    parser.add_argument("--limit", type=int, default=0, help="case count limit (0 means all)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    case_list = load_case_list(args.cases)
    if args.limit > 0:
        case_list = case_list[: args.limit]

    print("[1/4] 평가 문장을 모읍니다.")
    user_list = unique_text_list(case_list)

    print("[2/4] TTS/STT 모델을 올립니다.")
    tts_obj = TTS_Engine()
    stt_model = whisper.load_model(STT_MODEL, device="cuda")

    print("[3/4] 실제 답변 문장을 뽑습니다.")
    speech_rows = build_speech_rows(case_list)

    print("[4/4] 여러 설정을 자동으로 돌립니다.")
    stt_obj = run_stt_bench(tts_obj, stt_model, user_list)
    tts_obj_res = run_tts_bench(tts_obj, speech_rows)

    report_obj = {
        "meta": {
            "cases_path": args.cases,
            "case_count": len(case_list),
            "user_text_count": len(user_list),
            "speech_count": len(speech_rows),
            "stt_model": STT_MODEL,
            "classifier_model": CLASSIFIER_MODEL,
            "planner_model": PLANNER_MODEL,
        },
        "stt": stt_obj,
        "tts": tts_obj_res,
        "best_defaults": {
            "stt_use_fp16": stt_obj["best"]["fp16"],
            "stt_initial_prompt": stt_obj["best"]["prompt"],
            "tts_stream_enabled": tts_obj_res["best"]["stream"],
            "tts_stream_min_len": tts_obj_res["best"]["min_len"],
        },
    }
    out_path = save_report(report_obj)

    print("\n=== 추천 설정 ===")
    print(json.dumps(report_obj["best_defaults"], ensure_ascii=False, indent=2))
    print(f"\n리포트 저장: {out_path}")


if __name__ == "__main__":
    main()
