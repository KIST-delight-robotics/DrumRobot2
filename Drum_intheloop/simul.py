import argparse
import logging
import os
import select
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import can

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sil.decoder import decode_can_frame, decode_dxl_packet, dxl_goal_deg, dxl_read_ids, split_dxl_packets
from sil.encoder import (
    encode_dxl_ping,
    encode_dxl_read,
    encode_dxl_write,
    encode_maxon_sdo_ack,
    motor_feedback,
)
from sil.mapping import (
    CAN_BUS_MOTORS,
    DEFAULT_URDF_PATH,
    DXL_MOTORS,
    MAXON_SPEC,
    PRODUCTION_TO_URDF_JOINT,
    STARTUP_DXL_POSE_DEG,
    TMOTOR_SPEC,
    dxl_to_urdf_deg,
)
from sil.motor_state import NmtState
from sil.pybullet_backend import PyBulletBackend
from sil.router import MotorRouter

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger(__name__)


# 시뮬레이터 본체
class FrameSimulator:
    def __init__(
        self,
        can_buses,
        dxl_path: Path,
        urdf_path: Path,
        mode: str,
        feedback_hz: float,
    ):
        self.can_buses = list(can_buses)
        self.dxl_path = Path(dxl_path)
        self.feedback_dt = 1.0 / feedback_hz
        self.backend = PyBulletBackend(urdf_path=urdf_path, mode=mode)
        self.router = MotorRouter()
        self.nmt_state = NmtState()
        self.bus_map: Dict[str, object] = {}
        self.motor_bus: Dict[str, str] = self._default_motor_bus()
        self.dxl_feedback: Dict[int, float] = self._default_dxl_feedback()
        self.dxl_targets: Dict[str, float] = {}
        self.dxl_lock = threading.Lock()
        self.dxl_stop = threading.Event()
        self.dxl_thread: Optional[threading.Thread] = None
        self.dxl_fd: Optional[int] = None
        self.dxl_buffer = b""
        self.last_feedback = 0.0
        self.last_tmotor_command: Dict[str, float] = {}
        self.last_motion = 0.0
        self.needs_step = False

    # 생명주기
    def run(self) -> int:
        try:
            self.backend.start()
            self.backend.apply_targets(self.router.startup_targets())
            self.backend.step()
            self.last_motion = time.monotonic()
            self._open_can_buses()
            self._open_dxl()
            self._start_dxl_thread()

            while True:
                self._poll_can()
                self._apply_dxl_targets()
                self._advance_motion()
                if self.needs_step:
                    self.backend.step()
                    self.needs_step = False
                self._send_tmotor_idle_feedback()
                time.sleep(0.0005)
        except KeyboardInterrupt:
            return 0
        finally:
            self.close()

    def close(self) -> None:
        self.dxl_stop.set()
        if self.dxl_thread is not None:
            self.dxl_thread.join(timeout=0.5)
            self.dxl_thread = None

        if self.dxl_fd is not None:
            os.close(self.dxl_fd)
            self.dxl_fd = None

        for can_bus in self.bus_map.values():
            try:
                can_bus.shutdown()
            except Exception:
                pass

        self.bus_map.clear()
        self.backend.close()

    # 장치 초기화
    def _default_motor_bus(self) -> Dict[str, str]:
        motor_bus: Dict[str, str] = {}
        for can_bus, motors in CAN_BUS_MOTORS.items():
            if can_bus in self.can_buses:
                for motor in motors:
                    motor_bus[motor] = can_bus
        return motor_bus

    def _default_dxl_feedback(self) -> Dict[int, float]:
        feedback: Dict[int, float] = {}
        for dxl_id, motor in DXL_MOTORS.items():
            feedback[dxl_id] = STARTUP_DXL_POSE_DEG.get(motor, 0.0)
        return feedback

    def _open_can_buses(self) -> None:
        for can_bus in self.can_buses:
            self.bus_map[can_bus] = can.interface.Bus(
                channel=can_bus,
                interface="socketcan",
                receive_own_messages=False,
            )
            logger.info("[SIL] opened %s", can_bus)

    def _open_dxl(self) -> None:
        if not self.dxl_path.exists():
            logger.warning("[SIL] DXL PTY not found: %s", self.dxl_path)
            return

        self.dxl_fd = os.open(str(self.dxl_path), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        logger.info("[SIL] opened DXL PTY %s", self.dxl_path)

    def _start_dxl_thread(self) -> None:
        if self.dxl_fd is None:
            return

        self.dxl_thread = threading.Thread(target=self._dxl_loop, daemon=True)
        self.dxl_thread.start()

    def _dxl_loop(self) -> None:
        while not self.dxl_stop.is_set():
            self._poll_dxl()
            time.sleep(0.0002)

    # CAN 프레임 처리 반복
    def _poll_can(self) -> None:
        for can_bus, bus_obj in self.bus_map.items():
            latest_tmotor: Dict[str, float] = {}

            while True:
                frame = bus_obj.recv(timeout=0.0)
                if frame is None:
                    break

                command = decode_can_frame(frame)
                if command is None:
                    continue

                if command.kind.startswith("nmt_"):
                    self.nmt_state.transition(command.kind, command.motor)
                    continue

                if command.kind == "maxon_sync":
                    self._send_maxon_sync_feedback(can_bus)
                    continue

                if command.motor:
                    self.motor_bus[command.motor] = can_bus

                if command.kind == "maxon_sdo":
                    reply = encode_maxon_sdo_ack(command.motor)
                    if reply is not None:
                        bus_obj.send(reply)
                    continue

                targets = self.router.route_can(command)
                if targets:
                    self.backend.apply_targets(targets)
                    self.needs_step = True

                    # TMotor 실제 장치 모델:
                    # CAN queue를 비우는 동안 모터별 최신 target만 남겼다가 echo feedback으로 되돌려준다.
                    if command.motor in TMOTOR_SPEC:
                        for urdf_deg in targets.values():
                            latest_tmotor[command.motor] = urdf_deg

            if latest_tmotor:
                now = time.monotonic()
                for motor, urdf_deg in latest_tmotor.items():
                    self.last_tmotor_command[motor] = now
                    self._send_motor_feedback(motor, can_bus, urdf_deg)

    # DXL 패킷 처리 반복
    def _poll_dxl(self) -> None:
        if self.dxl_fd is None:
            return

        for _ in range(16):
            readable, _, _ = select.select([self.dxl_fd], [], [], 0)
            if not readable:
                return

            try:
                data = os.read(self.dxl_fd, 4096)
            except BlockingIOError:
                return

            if not data:
                return

            self.dxl_buffer += data
            packets, self.dxl_buffer = split_dxl_packets(self.dxl_buffer)
            for packet in packets:
                commands = decode_dxl_packet(packet)
                for command in commands:
                    self._handle_dxl(command)

    def _handle_dxl(self, command) -> None:
        if self.dxl_fd is None:
            return

        if command.kind == "ping":
            response = encode_dxl_ping(command.dxl_id)
            if response is not None:
                self._write_dxl(response)
            return

        if command.kind == "write":
            response = encode_dxl_write(command.dxl_id)
            if response is not None:
                self._write_dxl(response)
            return

        if command.kind == "sync_read":
            # DXL 실제 장치 모델:
            # 주기 feedback은 만들지 않는다. SyncWrite로 받은 최신 goal을 보관해 두었다가
            # SyncRead packet을 받을 때만 status packet으로 즉시 echo한다.
            for dxl_id in dxl_read_ids(command):
                joint_deg = self.dxl_feedback.get(dxl_id, 0.0)
                response = encode_dxl_read(dxl_id, joint_deg)
                if response is not None:
                    self._write_dxl(response)
            return

        if command.kind == "sync_write":
            goal_deg = dxl_goal_deg(command.data)
            if goal_deg is not None:
                self.dxl_feedback[command.dxl_id] = goal_deg
                self._stage_dxl_target(command.dxl_id, goal_deg)
            return

    def _stage_dxl_target(self, dxl_id: int, goal_deg: float) -> None:
        motor = DXL_MOTORS.get(dxl_id)
        if motor is None:
            return

        targets = dxl_to_urdf_deg(motor, goal_deg)
        if not targets:
            return

        with self.dxl_lock:
            self.dxl_targets.update(targets)

    def _write_dxl(self, packet: bytes) -> None:
        if self.dxl_fd is None:
            return

        view = memoryview(packet)
        while view:
            try:
                sent = os.write(self.dxl_fd, view)
            except BlockingIOError:
                time.sleep(0.0001)
                continue
            if sent == 0:
                return
            view = view[sent:]

    def _apply_dxl_targets(self) -> None:
        with self.dxl_lock:
            if not self.dxl_targets:
                return

            targets = dict(self.dxl_targets)
            self.dxl_targets.clear()

        self.backend.apply_targets(targets)
        self.needs_step = True

    # 모션/피드백 반복
    def _advance_motion(self) -> None:
        now = time.monotonic()
        dt = now - self.last_motion
        self.last_motion = now

        targets = self.router.advance(dt)
        if targets:
            self.backend.apply_targets(targets)
            self.needs_step = True

    def _send_tmotor_idle_feedback(self) -> None:
        # TMotor discovery/idle 모델:
        # DrumRobot2가 초기 연결 확인을 할 수 있도록 명령 전 TMotor에만 200Hz status를 유지한다.
        # command를 받은 뒤에는 target echo만 실제 feedback source로 둔다.
        now = time.monotonic()
        if now - self.last_feedback < self.feedback_dt:
            return

        state = self.backend.read_joint_states()
        for motor, can_bus in self.motor_bus.items():
            if motor not in TMOTOR_SPEC:
                continue

            if motor in self.last_tmotor_command:
                continue

            joint_name = PRODUCTION_TO_URDF_JOINT.get(motor)
            if joint_name is None:
                continue

            urdf_deg = state.get(joint_name)
            if urdf_deg is not None:
                self._send_motor_feedback(motor, can_bus, urdf_deg)

        self.last_feedback = now

    def _send_maxon_sync_feedback(self, can_bus: str) -> None:
        # Maxon 실제 장치 모델:
        # 200Hz 주기 feedback을 따로 뿌리지 않고, CANopen SYNC(0x80)를 받을 때
        # Operational 상태인 노드의 TPDO 위치 feedback만 전송한다.
        state = self.backend.read_joint_states()
        for motor, motor_bus in self.motor_bus.items():
            if motor_bus != can_bus or motor not in MAXON_SPEC:
                continue

            joint_name = PRODUCTION_TO_URDF_JOINT.get(motor)
            if joint_name is None:
                continue

            urdf_deg = state.get(joint_name)
            if urdf_deg is not None:
                self._send_motor_feedback(motor, can_bus, urdf_deg)

    def _send_motor_feedback(
        self,
        motor: Optional[str],
        can_bus: str,
        urdf_deg: float,
    ) -> None:
        if motor is None:
            return

        if motor in MAXON_SPEC and not self.nmt_state.is_operational(motor):
            return

        bus_obj = self.bus_map.get(can_bus)
        if bus_obj is None:
            return

        frame = motor_feedback(motor, urdf_deg)
        if frame is not None:
            bus_obj.send(frame)


# 명령줄 진입점
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run frame-level DrumRobot SIL.")
    parser.add_argument("--mode", choices=["gui", "direct"], default="gui")
    parser.add_argument("--dxl", type=Path, default=Path("/tmp/ttyUSB0_sim"))
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument(
        "--feedback-hz",
        type=float,
        default=200.0,
        help="TMotor idle/discovery status rate.",
    )
    parser.add_argument("can_buses", nargs="*", default=["vcan0", "vcan1", "vcan2", "vcan3"])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    simulator = FrameSimulator(
        can_buses=args.can_buses,
        dxl_path=args.dxl,
        urdf_path=args.urdf,
        mode=args.mode,
        feedback_hz=args.feedback_hz,
    )
    return simulator.run()


if __name__ == "__main__":
    raise SystemExit(main())
