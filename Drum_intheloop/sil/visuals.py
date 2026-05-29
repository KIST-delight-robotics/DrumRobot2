import math
from pathlib import Path
from typing import Dict, Optional

import pybullet as p
import pybullet_data

from .colors import PLANE_RGBA, ROBOT_THEME
from .mapping import (
    DRUM_HEAD_INDICES,
    DRUM_PAD_OFFSET,
    DRUM_PAD_SKIP_INDICES,
    DRUM_PAD_SPEC,
    PEDAL_SPEC,
)


# World setup
def setup_world(client_id: int, mode: str) -> None:
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client_id)

    if PLANE_RGBA is not None:
        p.changeVisualShape(plane_id, -1, rgbaColor=PLANE_RGBA, physicsClientId=client_id)

    if mode == "gui":
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client_id)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.8,
            cameraYaw=45.0,
            cameraPitch=-20.0,
            cameraTargetPosition=[0.0, 0.0, 0.85],
            physicsClientId=client_id,
        )


# Robot visuals
def apply_robot_theme(client_id: int, robot_id: int) -> None:
    base_color = ROBOT_THEME.get("base_link")
    if base_color is not None:
        p.changeVisualShape(
            robot_id,
            -1,
            rgbaColor=base_color,
            physicsClientId=client_id,
        )

    joint_count = p.getNumJoints(robot_id, physicsClientId=client_id)
    for joint_index in range(joint_count):
        joint_info = p.getJointInfo(robot_id, joint_index, physicsClientId=client_id)
        link_name = joint_info[12].decode("utf-8")
        link_color = ROBOT_THEME.get(link_name)
        if link_color is None:
            continue
        p.changeVisualShape(
            robot_id,
            joint_index,
            rgbaColor=link_color,
            physicsClientId=client_id,
        )


# Pedal visuals
def create_pedals(client_id: int) -> Dict[str, int]:
    return {
        "right": _create_pedal(client_id, PEDAL_SPEC["pos_right"], PEDAL_SPEC["color_right"]),
        "left": _create_pedal(client_id, PEDAL_SPEC["pos_left"], PEDAL_SPEC["color_left"]),
    }


def tilt_pedal(client_id: int, pedal_id: Optional[int], side: str, angle_deg: float) -> None:
    if pedal_id is None:
        return

    base_pos = list(_pedal_pos(side))
    tilt = max(0.0, min(abs(angle_deg) * 0.6, PEDAL_SPEC["max_tilt_deg"]))
    tilt_rad = math.radians(tilt)
    orn = p.getQuaternionFromEuler([0.0, -tilt_rad, 0.0])

    half_len = PEDAL_SPEC["half_extents"][1]
    pivot_x = half_len * (1.0 - math.cos(tilt_rad))
    pivot_z = half_len * math.sin(tilt_rad)
    pos = [
        base_pos[0] + pivot_x,
        base_pos[1],
        base_pos[2] + pivot_z * 0.5,
    ]

    p.resetBasePositionAndOrientation(
        pedal_id,
        pos,
        orn,
        physicsClientId=client_id,
    )


# Drum pad visuals
def add_drum_pads(client_id: int) -> None:
    positions = _load_drum_positions()
    if not positions:
        return

    height = DRUM_PAD_SPEC["height"]
    for index, pos in enumerate(positions):
        if index in DRUM_PAD_SKIP_INDICES:
            continue

        is_drum = index in DRUM_HEAD_INDICES
        radius = DRUM_PAD_SPEC["drum_radius_outer"] if is_drum else DRUM_PAD_SPEC["cymbal_radius_outer"]
        col = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=radius,
            height=height,
            physicsClientId=client_id,
        )
        vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=radius,
            length=height,
            rgbaColor=DRUM_PAD_SPEC["color_outer"],
            physicsClientId=client_id,
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=list(pos),
            physicsClientId=client_id,
        )


# Local helpers
def _create_pedal(client_id: int, pos, color) -> int:
    half = PEDAL_SPEC["half_extents"]
    col = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half,
        physicsClientId=client_id,
    )
    vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=color,
        physicsClientId=client_id,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=pos,
        physicsClientId=client_id,
    )


def _pedal_pos(side: str):
    if side == "right":
        return PEDAL_SPEC["pos_right"]
    return PEDAL_SPEC["pos_left"]


def _load_drum_positions() -> list:
    drum_pos_path = (
        Path(__file__).resolve().parents[2]
        / "DrumRobot2"
        / "include"
        / "drum"
        / "drum_position.txt"
    )
    if not drum_pos_path.exists():
        return []

    values: list = []
    with drum_pos_path.open() as file:
        for line in file:
            values.extend(float(value) for value in line.split())

    positions = []
    for col in range(10):
        rx_r = values[col]
        ry_r = values[10 + col]
        rz_r = values[20 + col]
        rx_l = values[30 + col]
        ry_l = values[40 + col]
        rz_l = values[50 + col]
        wx = (rx_r + rx_l) * 0.5 + DRUM_PAD_OFFSET[0]
        wy = (ry_r + ry_l) * 0.5 + DRUM_PAD_OFFSET[1]
        wz = (rz_r + rz_l) * 0.5 + DRUM_PAD_OFFSET[2]
        positions.append((wx, wy, wz))

    return positions
