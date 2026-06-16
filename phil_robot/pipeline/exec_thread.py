"""
Executor — 로봇 명령을 백그라운드 스레드에서 순서대로 전송한다.

wait/취소 개념은 제거됐다. 예전엔 wait 지연(`wait:<seconds>`)을 중간에 끊으려고
stop_event/cancel 이 있었지만, wait 가 사라지면서 명령은 즉시 전송되고 스레드도
곧 끝난다. 그래서 execute() 는 보내기만 하고, 다 보내면 on_done() 을 호출한다.

전송 자체는 거의 즉시라 사실 동기로 해도 무방하다(보내도 로봇이 실제로 움직이는 데
시간이 걸려 TTS 와의 체감 순서 차이가 없다). 비동기로 두는 건 큰 이득이라기보다
나중을 위한 여지 정도다.
"""

import threading
from typing import Callable, List, Optional


class Executor:

    def __init__(self, bot):
        self._bot = bot
        self._thread: Optional[threading.Thread] = None
        self._lock: threading.Lock = threading.Lock()

    def exec_cmd(self, commands: List[str], on_done: Callable[[], None]) -> None:
        """commands 를 백그라운드 스레드에서 순서대로 전송하고, 끝나면 on_done() 을 부른다."""
        with self._lock:
            self._thread = threading.Thread(
                target=self._run_commands,
                args=(commands, on_done),
                daemon=True,
            )
            self._thread.start()

    def is_running(self) -> bool:
        """백그라운드 전송 스레드가 아직 살아있으면 True."""
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def _run_commands(self, commands: List[str], on_done: Callable[[], None]) -> None:
        """백그라운드 스레드 본체: 명령을 순서대로 전송한 뒤 on_done() 을 호출한다."""
        for cmd in commands:
            print(f"📡 [Executor] 명령 전송: {cmd}")
            try:
                self._bot.send_command(cmd + "\n")
            except Exception as exc:
                print(f"⚠️ [Executor] 전송 실패: {exc}")

        try:
            on_done()
        except Exception as exc:
            print(f"⚠️ [Executor] on_done 콜백 오류: {exc}")
