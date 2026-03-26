#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


SDK_SRC = Path(__file__).resolve().parents[1] / "DynamixelSDK-3.8.4" / "python" / "src"
if str(SDK_SRC) not in sys.path:
    sys.path.insert(0, str(SDK_SRC))

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler  # type: ignore


ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

POSITION_MODE = 3
TORQUE_ON = 1
TORQUE_OFF = 0

MIN_TICK = 0
MAX_TICK = 4095
TICKS_PER_DEGREE = 4096.0 / 360.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move one DYNAMIXEL a small amount.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--protocol", type=float, default=2.0)
    parser.add_argument("--id", type=int, default=28)
    parser.add_argument("--degrees", type=float, default=10.0, help="Relative move size from current position.")
    parser.add_argument("--cycles", type=int, default=1, help="How many left/right cycles to run.")
    parser.add_argument("--settle", type=float, default=1.0, help="Seconds to wait after each move.")
    parser.add_argument("--profile-accel-ms", type=int, default=300)
    parser.add_argument("--profile-velocity-ms", type=int, default=600)
    parser.add_argument("--ping-retries", type=int, default=10)
    parser.add_argument("--retry-sleep", type=float, default=0.1)
    return parser.parse_args()


def open_port(port_name: str, baudrate: int) -> PortHandler:
    port = PortHandler(port_name)
    port.getCFlagBaud = lambda baud: baud

    if not port.openPort():
        raise RuntimeError(f"failed to open {port_name}")
    if not port.setBaudRate(baudrate):
        raise RuntimeError(f"failed to set baudrate {baudrate}")
    time.sleep(0.1)
    try:
        port.clearPort()
    except Exception:
        pass
    return port


def expect_ok(packet: PacketHandler, result: int, error: int, action: str) -> None:
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{action}: {packet.getTxRxResult(result)}")
    if error != 0:
        raise RuntimeError(f"{action}: {packet.getRxPacketError(error)}")


def read_present_position(port: PortHandler, packet: PacketHandler, dxl_id: int) -> int:
    try:
        port.clearPort()
    except Exception:
        pass
    value, result, error = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    expect_ok(packet, result, error, "read present position")
    return int(value)


def write_1byte(port: PortHandler, packet: PacketHandler, dxl_id: int, address: int, value: int, label: str) -> None:
    try:
        port.clearPort()
    except Exception:
        pass
    result, error = packet.write1ByteTxRx(port, dxl_id, address, value)
    expect_ok(packet, result, error, label)


def write_4byte(port: PortHandler, packet: PacketHandler, dxl_id: int, address: int, value: int, label: str) -> None:
    try:
        port.clearPort()
    except Exception:
        pass
    result, error = packet.write4ByteTxRx(port, dxl_id, address, int(value))
    expect_ok(packet, result, error, label)


def ping_with_retry(
    port: PortHandler,
    packet: PacketHandler,
    dxl_id: int,
    retries: int,
    retry_sleep: float,
) -> int:
    last_result = None
    last_error = None
    last_model = 0

    for attempt in range(1, retries + 1):
        try:
            port.clearPort()
        except Exception:
            pass
        model, result, error = packet.ping(port, dxl_id)
        if result == COMM_SUCCESS and error == 0:
            if attempt > 1:
                print(f"ping recovered on attempt {attempt}")
            return model
        last_model = model
        last_result = result
        last_error = error
        print(
            f"ping attempt {attempt}/{retries} failed: "
            f"result={packet.getTxRxResult(result)} error={error}"
        )
        time.sleep(retry_sleep)

    raise RuntimeError(
        f"ping failed after {retries} attempts: "
        f"model={last_model} result={packet.getTxRxResult(last_result)} error={last_error}"
    )


def delta_deg_to_ticks(delta_deg: float) -> int:
    return int(round(delta_deg * TICKS_PER_DEGREE))


def clamp_tick(value: int) -> int:
    return max(MIN_TICK, min(MAX_TICK, value))


def tick_to_degree(value: int) -> float:
    return (2048.0 - float(value)) * (360.0 / 4096.0)


def main() -> int:
    args = parse_args()

    port = open_port(args.port, args.baud)
    packet = PacketHandler(args.protocol)

    try:
        model = ping_with_retry(port, packet, args.id, args.ping_retries, args.retry_sleep)
        print(f"connected: id={args.id} model={model} baud={args.baud} protocol={args.protocol}")

        write_1byte(port, packet, args.id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "torque off")
        write_1byte(port, packet, args.id, ADDR_OPERATING_MODE, POSITION_MODE, "set position mode")
        write_4byte(port, packet, args.id, ADDR_PROFILE_ACCEL, args.profile_accel_ms, "set profile accel")
        write_4byte(port, packet, args.id, ADDR_PROFILE_VELOCITY, args.profile_velocity_ms, "set profile velocity")
        write_1byte(port, packet, args.id, ADDR_TORQUE_ENABLE, TORQUE_ON, "torque on")

        start_tick = read_present_position(port, packet, args.id)
        print(f"start: tick={start_tick} deg={tick_to_degree(start_tick):.2f}")

        delta_tick = delta_deg_to_ticks(args.degrees)
        targets = []
        for _ in range(args.cycles):
            targets.append(clamp_tick(start_tick + delta_tick))
            targets.append(clamp_tick(start_tick - delta_tick))
        targets.append(start_tick)

        for target in targets:
            print(f"move: tick={target} deg={tick_to_degree(target):.2f}")
            write_4byte(port, packet, args.id, ADDR_GOAL_POSITION, target, "write goal position")
            time.sleep(args.settle)

        final_tick = read_present_position(port, packet, args.id)
        print(f"final: tick={final_tick} deg={tick_to_degree(final_tick):.2f}")
        return 0
    finally:
        try:
            write_1byte(port, packet, args.id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "torque off")
        except Exception:
            pass
        port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
