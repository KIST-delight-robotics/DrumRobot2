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
START_THRESHOLD = 15       # 이 볼륨을 넘으면 발화 시작으로 본다.
STOP_THRESHOLD = 8         # 이 볼륨 아래는 침묵으로 센다.
PRE_RECORD_SECONDS = 0.5   # 시작 직전 오디오를 미리 담아 두는 길이.
MAX_SILENT_FRAMES = 12     # 1.2초 침묵이 이어지면 발화 종료.
MAX_UTTER_FRAMES = 100     # 10초를 넘으면 강제 종료.
MIN_UTTER_SECONDS = 1.0    # 이보다 짧은 발화는 잡음으로 보고 버린다.


class MicListener:
    """백그라운드에서 마이크를 계속 듣고, VAD로 끊은 발화 오디오를 큐에 넣는다.

    `tests/test_speech.py`의 listener thread 구조를 옮긴 것이다.
    - START_THRESHOLD를 넘으면 녹음을 시작한다.
    - STOP_THRESHOLD 아래 침묵이 MAX_SILENT_FRAMES만큼 이어지면 발화로 본다.
    - TTS 재생 중에는 `set_speaking(True)`로 청취를 막아 self-echo를 차단한다.
      (마이크와 스피커가 같은 공간에 있어, 막지 않으면 필 자신의 목소리를
       다음 발화로 주워 담는다.)

    메인 루프는 `read_utterance()`로 확정된 발화 오디오 한 건씩만 받아 간다.
    """

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
        pre_len = int(PRE_RECORD_SECONDS / 0.1)
        pre_buffer = deque(maxlen=pre_len)

        with sd.InputStream(samplerate=SAMPLE_RATE, channels=1) as stream:
            while self._running.is_set():
                try:
                    indata, _ = stream.read(FRAME_SIZE)
                    pre_buffer.append(indata)

                    # 필이 말하는 중에는 듣지 않는다(echo 방지).
                    if self._speaking.is_set():
                        sd.sleep(100)
                        continue

                    volume = np.linalg.norm(indata) * 10
                    if volume <= START_THRESHOLD:
                        continue

                    print(f"\n⚡ 소리 감지 (Vol: {volume:.1f}) -> 녹음 시작")
                    frames = self._record_until_silence(stream, pre_buffer)

                    if len(frames) * 0.1 > MIN_UTTER_SECONDS:
                        audio = np.concatenate(frames).flatten().astype(np.float32)
                        self._queue.put(audio)
                        print("📦 발화 확정 (Queue)")
                    else:
                        print("🧹 너무 짧아서 버림")

                except Exception as exc:
                    print(f"❌ 마이크 리스너 에러: {exc}")
                    sd.sleep(1000)

    def _record_until_silence(self, stream, pre_buffer):
        """침묵이 이어질 때까지 한 발화를 녹음해 frame 리스트로 반환한다."""
        frames = list(pre_buffer)
        silent_frames = 0

        # 녹음 도중 TTS가 시작되면(_speaking) 그 녹음에는 필 목소리가
        # 섞이므로 즉시 멈춰 버린다. 이것도 위 echo 방지 게이트의 연장이다.
        while self._running.is_set() and not self._speaking.is_set():
            data, _ = stream.read(FRAME_SIZE)
            frames.append(data)

            volume = np.linalg.norm(data) * 10
            if volume > STOP_THRESHOLD:
                silent_frames = 0
            else:
                silent_frames += 1

            if silent_frames > MAX_SILENT_FRAMES:
                break
            if len(frames) > MAX_UTTER_FRAMES:
                break

        return frames
