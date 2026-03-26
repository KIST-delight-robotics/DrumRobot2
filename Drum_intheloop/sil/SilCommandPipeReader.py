import argparse
import json
import os
import stat
import sys
import time
from pathlib import Path
from typing import Dict, Iterator

# Ensure the project root is in the Python path for imports
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sil.command_applier import CommandApplier
from sil.command_types import MaxonData, TMotorData
from sil.pybullet_backend import PyBulletBackend
from sil.robot_spec import STARTUP_CAN_POSE_DEG, STARTUP_DXL_POSE_DEG

# Writer 경로와 동일
DEFAULT_PIPE_PATH = Path("/tmp/drum_command.pipe")

# URDF 경로는 PyBulletBackend의 기본 URDF와 동일하게 설정
DEFAULT_URDF_PATH = (
    PROJECT_ROOT
    / "urdf"
    / "drumrobot_RL_urdf"
    / "urdf"
    / "drumrobot_RL_urdf.urdf"
)

# SilCommandPipeReader 클래스는 named pipe에서 명령 메시지를 읽고 파싱하는 역할을 담당
class SilCommandPipeReader:
    def __init__(self, pipe_path: Path = DEFAULT_PIPE_PATH):
        self.pipe_path = Path(pipe_path)
        self._owns_pipe = False

    # stale FIFO를 지우고 새 named pipe를 준비
    def prepare_pipe(self) -> None:
        if self.pipe_path.exists():
            path_stat = self.pipe_path.stat()
            if not stat.S_ISFIFO(path_stat.st_mode):
                raise RuntimeError(f"Pipe path exists but is not a FIFO: {self.pipe_path}")
            self.pipe_path.unlink()

        os.mkfifo(self.pipe_path)
        self._owns_pipe = True

    def cleanup_pipe(self) -> None:
        if not self._owns_pipe:
            return

        if self.pipe_path.exists():
            path_stat = self.pipe_path.stat()
            if stat.S_ISFIFO(path_stat.st_mode):
                self.pipe_path.unlink()

        self._owns_pipe = False

    # named pipe에서 메시지를 읽고 파싱된 명령을 생성하는 제너레이터
    def read_messages(self) -> Iterator[dict]:
        with self.pipe_path.open("r", encoding="utf-8") as pipe:
            for raw_line in pipe:
                line = raw_line.strip()
                if not line:
                    continue

                yield self.parse_line(line)

    # 메시지를 JSON파싱하고 kind에 따라 dict 구조로 변환
    def parse_line(self, line: str) -> dict:
        payload = json.loads(line)
        kind = payload["kind"]

        if kind == "tmotor":
            return {
                "kind": kind,
                "motor": payload["motor"],
                "command": TMotorData(
                    position=float(payload["position"]),
                    velocityERPM=float(payload["velocityERPM"]),
                    mode=int(payload["mode"]),
                    useBrake=int(payload["useBrake"]),
                ),
            }

        if kind == "maxon":
            return {
                "kind": kind,
                "motor": payload["motor"],
                "command": MaxonData(
                    position=float(payload["position"]),
                    mode=int(payload["mode"]),
                    kp=int(payload["kp"]),
                    kd=int(payload["kd"]),
                ),
            }

        if kind == "dxl":
            return {
                "kind": kind,
                "motor": payload["motor"],
                "position": float(payload["position"]),
            }

        raise ValueError(f"Unsupported command kind: {kind}")

# command_applier에 넘겨서 URDF joint dict 만듬
def build_joint_targets(applier: CommandApplier, message: dict) -> Dict[str, float]:
    if message["kind"] in {"tmotor", "maxon"}:
        return applier.apply_can_command(message["motor"], message["command"])

    if message["kind"] == "dxl":
        return applier.apply_dxl_command(message["motor"], message["position"])

    return {}


def build_startup_joint_targets(applier: CommandApplier) -> Dict[str, float]:
    joint_targets_deg: Dict[str, float] = {}

    for motor_name, target_deg in STARTUP_CAN_POSE_DEG.items():
        command = applier.build_default_can_command(motor_name, target_deg)
        joint_targets_deg.update(applier.apply_can_command(motor_name, command))

    for motor_name, target_deg in STARTUP_DXL_POSE_DEG.items():
        joint_targets_deg.update(applier.apply_dxl_command(motor_name, target_deg))

    return joint_targets_deg

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read command-level SIL messages from a named pipe and apply them to PyBullet."
    )
    parser.add_argument(
        "--pipe",
        type=Path,
        default=DEFAULT_PIPE_PATH,
        help="Named pipe path. Must match the C++ SilCommandPipeWriter path.",
    )
    parser.add_argument(
        "--mode",
        default="gui",
        choices=["gui", "direct"],
        help="PyBullet connection mode.",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="URDF file to load.",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0001,
        help="Optional delay after each applied message.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    reader = SilCommandPipeReader(args.pipe)
    applier = CommandApplier()
    backend = PyBulletBackend(urdf_path=args.urdf, mode=args.mode)

    try:
        reader.prepare_pipe()
        backend.start()
        startup_joint_targets_deg = build_startup_joint_targets(applier)
        if startup_joint_targets_deg:
            print(f"Applying startup preset pose: {startup_joint_targets_deg}")
            backend.apply_targets(startup_joint_targets_deg)
            backend.step()
        print(f"Listening on named pipe: {args.pipe}")

        for message in reader.read_messages():
            joint_targets_deg = build_joint_targets(applier, message)
            if not joint_targets_deg:
                continue

            print(f"Received: {message}")
            print(f"Applying: {joint_targets_deg}")
            backend.apply_targets(joint_targets_deg)
            backend.step()

            if args.sleep > 0.0:
                time.sleep(args.sleep)

        return 0
    finally:
        backend.close()
        reader.cleanup_pipe()


if __name__ == "__main__":
    raise SystemExit(main())
