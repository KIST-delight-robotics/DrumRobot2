# phil_robot/runtime/mic_listener.py

import queue
import threading
from collections import deque

import numpy as np
import sounddevice as sd

# ==========================================
# VAD / 녹음 파라미터
# ==========================================
SAMPLE_RATE = 16000
FRAME_SIZE = 1600          # 0.1초 단위로 읽는다.
FRAME_SECONDS = FRAME_SIZE / SAMPLE_RATE
START_THRESHOLD = 15       # 이 볼륨을 넘으면 발화 시작으로 본다.
STOP_THRESHOLD = 8         # 이 볼륨 아래는 침묵으로 센다.
PRE_RECORD_SECONDS = 0.5   # 시작 직전 오디오를 미리 담아 두는 길이.
MAX_SILENT_FRAMES = 12     # 1.2초 침묵이 이어지면 발화 종료.
MAX_UTTER_FRAMES = 200     # 20초를 넘으면 강제 종료.
MIN_CLIP_SECONDS = 1.0     # 최종 오디오 클립이 이보다 짧으면 잡음으로 본다.


class MicListener:
    """백그라운드에서 발화 단위 오디오를 만들어 큐에 넣는다."""

    def __init__(self):
        self._queue = queue.Queue()
        self._speaking = threading.Event()
        self._running = threading.Event()
        self._thread = None

    def start(self):
        """리스너 스레드를 시작한다."""
        self._running.set()
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        print("👂 마이크 리스너 시작 (말이 끝날 때까지 자동 청취)")

    def set_speaking(self, speaking):
        """TTS 재생 중이면 True로 두어 청취를 막는다(echo 방지)."""
        if speaking:
            self._speaking.set()
        else:
            self._speaking.clear()

    def read_utterance(self, timeout=1.0):
        """확정된 발화 오디오 한 건을 반환한다. 없으면 None."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        """리스너 스레드를 정리한다."""
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ──────────────────────────────────────────────────────────────────────
    # 내부 구현
    # ──────────────────────────────────────────────────────────────────────
    def _listen_loop(self):
        pre_len = int(PRE_RECORD_SECONDS / FRAME_SECONDS)

        # 감지 직전 0.5초 오디오를 보관한다.
        pre_buffer = deque(maxlen=pre_len)

        with sd.InputStream(samplerate=SAMPLE_RATE, channels=1) as stream:
            while self._running.is_set():
                try:
                    # 마이크에서 0.1초 오디오를 읽는다.
                    indata, _ = stream.read(FRAME_SIZE)

                    # 감지 전 오디오를 계속 갱신한다.
                    pre_buffer.append(indata)

                    # TTS 중에는 입력을 버린다.
                    if self._speaking.is_set():
                        sd.sleep(100)
                        continue

                    # 0.1초마다 일정 볼륨 이상인지 확인 후 녹음 시작
                    volume = np.linalg.norm(indata) * 10
                    if volume < START_THRESHOLD:
                        continue

                    print(f"\n⚡ 소리 감지 (Vol: {volume:.1f}) -> 녹음 시작")
                    frames = self._record_until_silence(stream, pre_buffer)

                    if len(frames) * FRAME_SECONDS > MIN_CLIP_SECONDS:
                        audio = np.concatenate(frames).flatten().astype(np.float32)
                        self._queue.put(audio)
                        print("📦 발화 확정 (Queue)")
                    else:
                        print("🧹 너무 짧아서 버림")

                except Exception as exc:
                    print(f"❌ 마이크 리스너 에러: {exc}")
                    sd.sleep(1000)

    def _record_until_silence(self, stream, pre_buffer):
        """침묵이 이어질 때까지 한 발화를 녹음한다."""
        # 감지 직전 0.5초를 발화 앞에 붙인다.
        frames = list(pre_buffer)
        silent_frames = 0

        # TTS가 시작되면 현재 녹음을 중단한다.
        while self._running.is_set() and not self._speaking.is_set():
            # 발화 시작 이후의 0.1초 오디오를 읽는다.
            data, _ = stream.read(FRAME_SIZE)

            # 최종 발화 클립에 오디오 조각을 추가한다.
            frames.append(data)

            # 침묵이 얼마나 이어지는지 센다.
            volume = np.linalg.norm(data) * 10
            if volume > STOP_THRESHOLD:
                silent_frames = 0
            else:
                silent_frames += 1

            # 침묵이 충분히 길면 발화 종료로 본다.
            if silent_frames > MAX_SILENT_FRAMES:
                break

            # 너무 긴 발화는 강제로 끊는다.
            if len(frames) > MAX_UTTER_FRAMES:
                break

        return frames
