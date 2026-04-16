from dataclasses import dataclass
import copy
import time
from threading import Lock
from typing import Dict, List, Optional, Tuple

from .joint_map import (
    HOME_JOINT_ANGLES_DEG,
    INITIAL_JOINT_ANGLES_DEG,
    JOINT_LIMITS_DEG,
    SUPPORTED_GESTURES,
    clamp_angle,
    get_gesture_sequence,
)
from .protocol import serialize_state


@dataclass
class ScheduledPose:
    execute_at: float
    pose: Dict[str, float]
    look: Optional[Tuple[float, float]] = None


class SimulationState:
    def __init__(self):
        self._lock = Lock()
        self._current_angles = dict(INITIAL_JOINT_ANGLES_DEG)
        self._head_pan_deg = 0.0
        self._head_tilt_deg = 90.0
        self._pending: List[ScheduledPose] = []
        self._play_started_at: Optional[float] = None
        self._play_until: Optional[float] = None
        self._current_song = "None"
        self._last_action = "None"
        self._error_message = "None"
        self._led = "idle"

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            state_value = 2 if self._play_until and time.monotonic() < self._play_until else 0
            progress = self._build_progress(time.monotonic())
            return {
                "state": state_value,
                "bpm": 100,
                "is_fixed": True,
                "current_song": self._current_song,
                "progress": progress,
                "is_lock_key_removed": True,
                "last_action": self._last_action,
                "current_angles": copy.deepcopy(self._current_angles),
                "error_message": self._error_message,
                "sim_led": self._led,
                "sim_head": {
                    "pan": round(self._head_pan_deg, 2),
                    "tilt": round(self._head_tilt_deg, 2),
                },
            }

    def snapshot_json(self) -> str:
        return serialize_state(self.snapshot())

    def apply_named_pose(self, pose: Dict[str, float], action_name: str) -> None:
        with self._lock:
            for joint_name, angle_deg in pose.items():
                if joint_name not in self._current_angles:
                    continue
                self._current_angles[joint_name] = round(clamp_angle(joint_name, angle_deg), 2)
            self._last_action = action_name

    def set_joint(self, joint_name: str, angle_deg: float) -> None:
        with self._lock:
            if joint_name not in self._current_angles:
                return
            self._current_angles[joint_name] = round(clamp_angle(joint_name, angle_deg), 2)
            self._last_action = f"move:{joint_name},{angle_deg}"

    def set_look(self, pan_deg: float, tilt_deg: float) -> None:
        with self._lock:
            self._head_pan_deg = round(max(-90.0, min(90.0, pan_deg)), 2)
            self._head_tilt_deg = round(max(0.0, min(135.0, tilt_deg)), 2)
            self._last_action = f"look:{pan_deg},{tilt_deg}"

    def set_led(self, emotion: str) -> None:
        with self._lock:
            self._led = emotion
            self._last_action = f"led:{emotion}"

    def reset_ready(self) -> None:
        self.apply_named_pose(dict(INITIAL_JOINT_ANGLES_DEG), "r")

    def reset_home(self) -> None:
        self.apply_named_pose(dict(HOME_JOINT_ANGLES_DEG), "h")

    def stop(self) -> None:
        with self._lock:
            self._play_started_at = None
            self._play_until = None
            self._current_song = "None"
            self._last_action = "s"
            self._pending.clear()

    def start_play(self, song_code: str, duration_sec: float = 5.0) -> None:
        now = time.monotonic()
        with self._lock:
            self._play_started_at = now
            self._play_until = now + duration_sec
            self._current_song = song_code
            self._last_action = f"p:{song_code}"

    def queue_gesture(self, gesture_name: str) -> bool:
        gesture_key = (gesture_name or "").strip().lower()
        if gesture_key not in SUPPORTED_GESTURES:
            return False

        sequence = get_gesture_sequence(gesture_key)
        now = time.monotonic()
        with self._lock:
            for delay_sec, pose, look in sequence:
                self._pending.append(
                    ScheduledPose(
                        execute_at=now + delay_sec,
                        pose=dict(pose),
                        look=look,
                    )
                )
            self._pending.sort(key=lambda item: item.execute_at)
            self._last_action = f"gesture:{gesture_name}"
        return True

    def tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            while self._pending and self._pending[0].execute_at <= now:
                item = self._pending.pop(0)
                for joint_name, angle_deg in item.pose.items():
                    if joint_name in self._current_angles:
                        self._current_angles[joint_name] = round(clamp_angle(joint_name, angle_deg), 2)
                if item.look:
                    pan_deg, tilt_deg = item.look
                    self._head_pan_deg = round(max(-90.0, min(90.0, pan_deg)), 2)
                    self._head_tilt_deg = round(max(0.0, min(135.0, tilt_deg)), 2)

            if self._play_until and now >= self._play_until:
                self._play_started_at = None
                self._play_until = None
                self._current_song = "None"
                self._last_action = "play_complete"

    def get_render_targets(self) -> Tuple[Dict[str, float], float, float]:
        with self._lock:
            return (
                copy.deepcopy(self._current_angles),
                self._head_pan_deg,
                self._head_tilt_deg,
            )

    def _build_progress(self, now_monotonic: float) -> str:
        if not self._play_until or not self._play_started_at:
            return "0/1"

        total = max(0.001, self._play_until - self._play_started_at)
        ratio = (now_monotonic - self._play_started_at) / total
        ratio = max(0.0, min(1.0, ratio))
        if ratio >= 1.0:
            return "1/1"
        return f"{ratio:.2f}/1"
