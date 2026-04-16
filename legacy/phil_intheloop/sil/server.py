import argparse
import socket
import time
from pathlib import Path
from typing import Optional, TYPE_CHECKING

from .joint_map import get_home_pose, get_ready_pose
from .protocol import split_lines, unpack_payload
from .state_model import SimulationState

if TYPE_CHECKING:
    from .pybullet_backend import PyBulletBackend


DEFAULT_URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "urdf"
    / "drumrobot_RL_urdf"
    / "urdf"
    / "drumrobot_RL_urdf.urdf"
)


class SimulationServer:
    def __init__(self, host: str, port: int, mode: str, urdf_path: Path):
        from .pybullet_backend import PyBulletBackend

        self._host = host
        self._port = port
        self._backend = PyBulletBackend(urdf_path=urdf_path, mode=mode)
        self._state = SimulationState()
        self._server_socket: Optional[socket.socket] = None
        self._client_socket: Optional[socket.socket] = None
        self._client_buffer = ""

    def run(self, demo: Optional[str], frames: Optional[int], screenshot: Optional[Path]) -> None:
        self._backend.start()
        self._state.reset_ready()

        if demo:
            self._apply_demo(demo)

        self._open_socket()

        try:
            self._loop(max_frames=frames, screenshot=screenshot)
        finally:
            self._close()

    def _loop(self, max_frames: Optional[int], screenshot: Optional[Path]) -> None:
        last_broadcast = ""
        frame_count = 0

        while True:
            self._state.tick()
            self._accept_if_needed()
            self._receive_commands()

            body_targets, head_pan_deg, head_tilt_deg = self._state.get_render_targets()
            self._backend.apply_targets(body_targets, head_pan_deg, head_tilt_deg)
            self._backend.step()

            if self._client_socket is not None:
                payload = self._state.snapshot_json()
                if payload != last_broadcast:
                    self._send_line(payload)
                    last_broadcast = payload

            time.sleep(1.0 / 120.0)
            frame_count += 1

            if max_frames is not None and frame_count >= max_frames:
                if screenshot is not None:
                    self._backend.save_screenshot(screenshot)
                break

    def _open_socket(self) -> None:
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._server_socket.bind((self._host, self._port))
        self._server_socket.listen(1)
        self._server_socket.settimeout(0.01)
        print(f"[SIL] listening on {self._host}:{self._port}")

    def _accept_if_needed(self) -> None:
        if self._server_socket is None or self._client_socket is not None:
            return

        try:
            client_socket, client_address = self._server_socket.accept()
        except socket.timeout:
            return

        client_socket.settimeout(0.01)
        self._client_socket = client_socket
        self._client_buffer = ""
        print(f"[SIL] client connected from {client_address[0]}:{client_address[1]}")

    def _receive_commands(self) -> None:
        if self._client_socket is None:
            return

        try:
            data = self._client_socket.recv(4096)
        except socket.timeout:
            return
        except OSError:
            self._drop_client()
            return

        if not data:
            self._drop_client()
            return

        self._client_buffer += data.decode("utf-8", errors="ignore")
        lines, self._client_buffer = split_lines(self._client_buffer)
        for line in lines:
            for command in unpack_payload(line):
                self._handle_command(command)

    def _handle_command(self, command: str) -> None:
        clean = (command or "").strip()
        if not clean:
            return

        print(f"[SIL] command: {clean}")

        if clean == "r":
            self._state.reset_ready()
            return
        if clean == "h":
            self._state.reset_home()
            return
        if clean == "s":
            self._state.stop()
            self._state.apply_named_pose(get_home_pose(), "s")
            return
        if clean == "t":
            self._state.apply_named_pose(get_ready_pose(), "t")
            return
        if clean.startswith("p:"):
            _, _, song_code = clean.partition(":")
            self._state.reset_ready()
            self._state.start_play(song_code=song_code, duration_sec=5.0)
            return
        if clean.startswith("move:"):
            _, _, payload = clean.partition(":")
            if "," not in payload:
                return
            joint_name, angle_raw = payload.split(",", 1)
            try:
                angle_deg = float(angle_raw)
            except ValueError:
                return
            self._state.set_joint(joint_name, angle_deg)
            return
        if clean.startswith("look:"):
            _, _, payload = clean.partition(":")
            if "," not in payload:
                return
            pan_raw, tilt_raw = payload.split(",", 1)
            try:
                pan_deg = float(pan_raw)
                tilt_deg = float(tilt_raw)
            except ValueError:
                return
            self._state.set_look(pan_deg, tilt_deg)
            return
        if clean.startswith("gesture:"):
            _, _, gesture_name = clean.partition(":")
            self._state.queue_gesture(gesture_name)
            return
        if clean.startswith("led:"):
            _, _, emotion = clean.partition(":")
            self._state.set_led(emotion)
            return

    def _send_line(self, payload: str) -> None:
        if self._client_socket is None:
            return
        try:
            self._client_socket.sendall((payload + "\n").encode("utf-8"))
        except OSError:
            self._drop_client()

    def _drop_client(self) -> None:
        if self._client_socket is not None:
            try:
                self._client_socket.close()
            except OSError:
                pass
        self._client_socket = None
        self._client_buffer = ""
        print("[SIL] client disconnected")

    def _close(self) -> None:
        self._drop_client()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        self._backend.close()

    def _apply_demo(self, demo_name: str) -> None:
        name = demo_name.strip().lower()
        if name == "ready":
            self._state.reset_ready()
            return
        if name == "home":
            self._state.reset_home()
            return
        if name == "wave":
            self._state.queue_gesture("wave")
            return
        if name == "nod":
            self._state.queue_gesture("nod")
            return
        if name == "shake":
            self._state.queue_gesture("shake")
            return
        if name == "happy":
            self._state.queue_gesture("happy")
            return
        if name == "look_left":
            self._state.set_look(-30.0, 90.0)
            return
        if name == "look_right":
            self._state.set_look(30.0, 90.0)
            return


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Minimal PyBullet-based SIL server for Phil.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9999)
    parser.add_argument("--mode", choices=("gui", "direct"), default="gui")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument(
        "--demo",
        choices=("ready", "home", "wave", "nod", "shake", "happy", "look_left", "look_right"),
        default=None,
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="이 값이 주어지면 해당 프레임 수만큼만 실행하고 종료한다. 스크린샷용.",
    )
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="PPM 이미지 저장 경로. 보통 --frames 와 함께 쓴다.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    server = SimulationServer(
        host=args.host,
        port=args.port,
        mode=args.mode,
        urdf_path=args.urdf,
    )
    server.run(demo=args.demo, frames=args.frames, screenshot=args.screenshot)


if __name__ == "__main__":
    main()
