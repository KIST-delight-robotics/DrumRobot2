# 파일명: melo_engine.py
import os
import queue
import re
import sys
import threading
import time

import sounddevice as sd
import torch

THIRD_PARTY_MELO_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "third_party",
    "MeloTTS",
)
THIRD_PARTY_TORCHAUDIO_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "third_party",
    "downloads_whl",
    "audio",
)

if THIRD_PARTY_MELO_DIR not in sys.path:
    sys.path.insert(0, THIRD_PARTY_MELO_DIR)

if THIRD_PARTY_TORCHAUDIO_DIR not in sys.path:
    sys.path.insert(0, THIRD_PARTY_TORCHAUDIO_DIR)

from melo.api import TTS

ARTIFACT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "artifacts")
DIGIT_WORDS = {
    "0": "영",
    "1": "일",
    "2": "이",
    "3": "삼",
    "4": "사",
    "5": "오",
    "6": "육",
    "7": "칠",
    "8": "팔",
    "9": "구",
}
SMALL_UNITS = ["", "십", "백", "천"]
LARGE_UNITS = ["", "만", "억", "조"]

DECIMAL_NUMBER_PATTERN = re.compile(r"(?<![\d])(-?\d+\.\d+)(?![\d])")
INTEGER_NUMBER_PATTERN = re.compile(r"(?<![\d.])(-?\d+)(?![\d.])")

TEXT_REPLACEMENTS = {
    "MeloTTS": "멜로 티티에스",
    "Jetson": "젯슨",
    "CUDA": "쿠다",
    "GPU": "지피유",
    "CPU": "씨피유",
    "LLM": "엘엘엠",
    "BPM": "비피엠",
    "AI": "에이아이",
    "Orin": "오린",
    "Nano": "나노",
}


def _chunk_to_korean(chunk_text):
    """4자리 이하 숫자 조각을 한자어 수 읽기로 변환한다."""
    value = int(chunk_text)
    if value == 0:
        return ""

    result_parts = []
    reversed_digits = list(reversed(chunk_text))

    for index, digit in enumerate(reversed_digits):
        if digit == "0":
            continue

        unit = SMALL_UNITS[index]
        # 십/백/천 자리의 1은 '일십' 대신 '십'처럼 읽는다.
        if digit == "1" and unit:
            result_parts.append(unit)
        else:
            result_parts.append(DIGIT_WORDS[digit] + unit)

    return "".join(reversed(result_parts))


def number_to_korean_integer(number_text):
    """정수 문자열을 한자어 수 읽기로 변환한다."""
    normalized = str(number_text).strip()
    if not normalized:
        return normalized

    negative_prefix = ""
    if normalized.startswith("-"):
        negative_prefix = "마이너스 "
        normalized = normalized[1:]

    if not normalized.isdigit():
        return str(number_text)

    value = int(normalized)
    if value == 0:
        return negative_prefix + DIGIT_WORDS["0"]

    groups = []
    while normalized:
        groups.append(normalized[-4:])
        normalized = normalized[:-4]

    parts = []
    for index, group in enumerate(groups):
        chunk_text = _chunk_to_korean(group)
        if not chunk_text:
            continue
        parts.append(chunk_text + LARGE_UNITS[index])

    return negative_prefix + "".join(reversed(parts))


def decimal_to_korean(number_text):
    """소수 문자열을 '점' 발음으로 변환한다. 문장 마침표는 건드리지 않는다."""
    normalized = str(number_text).strip()
    if not normalized or "." not in normalized:
        return number_to_korean_integer(normalized)

    negative_prefix = ""
    if normalized.startswith("-"):
        negative_prefix = "마이너스 "
        normalized = normalized[1:]

    integer_part, fractional_part = normalized.split(".", 1)
    integer_reading = number_to_korean_integer(integer_part or "0")
    fractional_reading = "".join(DIGIT_WORDS[digit] for digit in fractional_part if digit in DIGIT_WORDS)

    if not fractional_reading:
        return negative_prefix + integer_reading

    return negative_prefix + f"{integer_reading}점{fractional_reading}"


class TTS_Engine:
    def __init__(self):
        print("\n[TTS] 엔진 시동 거는 중... (모델 로딩)")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        try:
            # 최초 1회 모델 로딩
            self.model = TTS(language="KR", device=self.device)
            self.speaker_ids = self.model.hps.data.spk2id
            self.speaker_id = self.speaker_ids["KR"]
            self.sample_rate = self.model.hps.data.sampling_rate
            print(f"[TTS] 로딩 완료 (장치: {self.device})")

            # 첫 합성 버벅임 완화를 위한 워밍업
            print("[TTS] 성대 푸는 중 (Warm-up)...")
            self.speak("준비 완료", play=False)

        except Exception as exc:
            print(f"[TTS] ❌ 치명적 오류: {exc}")
            self.model = None

    def preprocess(self, text):
        """TTS가 읽기 어려운 기술 용어와 숫자 표현을 한국어 발화 친화형으로 바꾼다."""
        normalized = str(text or "")

        for source, target in TEXT_REPLACEMENTS.items():
            normalized = normalized.replace(source, target).replace(source.lower(), target)

        # 소수는 먼저 처리해서 88.4 -> 팔십팔점사 로 바꾼다.
        normalized = DECIMAL_NUMBER_PATTERN.sub(
            lambda match: decimal_to_korean(match.group(1)),
            normalized,
        )

        # 남은 정수만 변환한다. 문장 마침표는 패턴에 걸리지 않는다.
        normalized = INTEGER_NUMBER_PATTERN.sub(
            lambda match: number_to_korean_integer(match.group(1)),
            normalized,
        )

        return normalized

    def _split_stream_text(self, clean_text):
        """구두점 기준으로 끊어서 스트리밍용 chunk 를 만든다."""
        word_list = [word.strip() for word in re.split(r"[.?!,]+", clean_text) if word.strip()]
        if len(word_list) >= 2:
            return word_list
        return [clean_text]

    def _render_audio(self, text):
        return self.model.tts_to_file(
            text,
            self.speaker_id,
            output_path=None,
            speed=1.0,
            quiet=True,
        )

    def _play_audio(self, audio_data):
        sd.play(audio_data, self.sample_rate)
        sd.wait()

    def _speak_file(self, clean_text, output_path=None, play=True):
        if output_path is None:
            os.makedirs(ARTIFACT_DIR, exist_ok=True)
            output_path = os.path.join(ARTIFACT_DIR, "temp_speech.wav")

        inference_start = time.time()
        self.model.tts_to_file(clean_text, self.speaker_id, output_path, speed=1.0)
        inference_end = time.time()
        print(f"  └ [TTS Inference] {inference_end - inference_start:.2f}s (텍스트 → 오디오 파일)")

        if play:
            play_start = time.time()
            os.system(f"aplay -q {output_path}")
            play_end = time.time()
            print(f"  └ [TTS Playback] {play_end - play_start:.2f}s (오디오 장치 재생)")

    def _speak_stream(self, clean_text):
        chunk_list = self._split_stream_text(clean_text)
        if len(chunk_list) < 2:
            self._speak_file(clean_text, play=True)
            return

        audio_q = queue.Queue(maxsize=2)
        stop_sig = object()
        error_box = {"exc": None}

        def synth_worker():
            try:
                for chunk_text in chunk_list:
                    synth_start = time.time()
                    audio_data = self._render_audio(chunk_text)
                    synth_sec = time.time() - synth_start
                    audio_q.put((chunk_text, audio_data, synth_sec))
            except Exception as exc:
                error_box["exc"] = exc
            finally:
                audio_q.put(stop_sig)

        worker = threading.Thread(target=synth_worker, daemon=True)
        worker.start()

        stream_start = time.time()
        first_audio_sec = None
        total_synth_sec = 0.0
        total_play_sec = 0.0
        chunk_count = 0

        while True:
            item = audio_q.get()
            if item is stop_sig:
                break

            _, audio_data, synth_sec = item
            chunk_count += 1
            total_synth_sec += synth_sec

            if first_audio_sec is None:
                first_audio_sec = time.time() - stream_start

            play_start = time.time()
            self._play_audio(audio_data)
            total_play_sec += time.time() - play_start

        worker.join()

        if error_box["exc"] is not None:
            raise error_box["exc"]

        if first_audio_sec is None:
            first_audio_sec = time.time() - stream_start

        print(
            f"  └ [TTS Stream] first_audio={first_audio_sec:.2f}s, "
            f"chunks={chunk_count}, synth_total={total_synth_sec:.2f}s, "
            f"play_total={total_play_sec:.2f}s"
        )

    def speak(self, text, output_path=None, play=True, stream=False):
        if not self.model:
            return

        clean_text = self.preprocess(text)
        if stream and play and output_path is None:
            try:
                self._speak_stream(clean_text)
                return
            except Exception as exc:
                print(f"  └ [TTS Stream Fallback] {exc}")

        self._speak_file(clean_text, output_path=output_path, play=play)
