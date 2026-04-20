"""
InterruptibleExecutor — 로봇 명령을 백그라운드 스레드에서 실행한다.

주요 동작:
- run()  : 명령 목록을 별도 스레드에서 순서대로 전송한다.
           wait: 명령은 짧은 loop(0.05s 단위)로 stop_event를 확인하면서 대기한다.
- cancel(): stop_event를 설정해 실행 스레드를 중단하고, 로봇에 'stop' 명령을 보낸다.
- on_done: 완료(cancelled=False) 또는 취소(cancelled=True) 후 호출되는 콜백.
           phil_brain.py 에서 홈 복귀 트리거로 사용한다.
"""

import threading
import time
from typing import Callable, List, Optional


class InterruptibleExecutor:

    def __init__(self, bot):
        self._bot = bot
        self._stop: threading.Event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock: threading.Lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, commands: List[str], on_done: Callable[[bool], None]) -> None:
        """
        commands 를 백그라운드 스레드에서 순서대로 실행한다.

        이전 실행이 아직 살아있으면 먼저 cancel() 을 호출해야 한다.
        중복 실행을 막기 위해 is_running() 확인 후 호출을 권장한다.

        on_done(cancelled: bool)
          - cancelled=False : 모든 명령이 정상 완료됨
          - cancelled=True  : cancel() 로 중단됨
        """
        with self._lock:
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run_commands,
                args=(commands, on_done),
                daemon=True,
            )
            self._thread.start()

    def cancel(self) -> None:
        """
        실행 중인 명령 시퀀스를 중단한다.
        1. stop_event 를 설정해 실행 스레드가 다음 체크 포인트에서 빠져나오게 한다.
        2. 로봇에 's' 명령을 전송해 현재 모션을 즉시 정지시킨다.
        """
        self._stop.set()
        try:
            self._bot.send_command("s\n")
        except Exception:
            pass  # 소켓이 닫혀 있어도 cancel 자체는 계속 진행한다

    def is_running(self) -> bool:
        """백그라운드 실행 스레드가 아직 살아있으면 True."""
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run_commands(self, commands: List[str], on_done: Callable[[bool], None]) -> None:
        """
        백그라운드 스레드 본체.
        stop_event 가 설정되면 현재 단계에서 즉시 빠져나온다.
        """
        cancelled = False

        for cmd in commands:
            if self._stop.is_set():
                cancelled = True
                break

            if cmd.startswith("wait:"):
                cancelled = self._interruptible_wait(cmd)
                if cancelled:
                    break
                continue

            print(f"📡 [Executor] 명령 전송: {cmd}")
            try:
                self._bot.send_command(cmd + "\n")
            except Exception as exc:
                print(f"⚠️ [Executor] 전송 실패: {exc}")

        try:
            on_done(cancelled)
        except Exception as exc:
            print(f"⚠️ [Executor] on_done 콜백 오류: {exc}")

    def _interruptible_wait(self, cmd: str) -> bool:
        """
        'wait:<seconds>' 명령을 처리한다.
        0.05초 단위로 stop_event 를 확인해 cancel() 이 들어오면 즉시 종료한다.

        반환값: True = 중단됨, False = 정상 완료
        """
        _, _, seconds_raw = cmd.partition(":")
        try:
            total_seconds = float(seconds_raw)
        except ValueError:
            return False

        print(f"⏳ [Executor] 대기: {total_seconds:.2f}초")
        deadline = time.monotonic() + total_seconds
        poll_interval = 0.05

        while time.monotonic() < deadline:
            if self._stop.is_set():
                return True
            remaining = deadline - time.monotonic()
            time.sleep(min(poll_interval, max(0.0, remaining)))

        return False
