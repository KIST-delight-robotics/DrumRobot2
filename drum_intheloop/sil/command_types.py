from dataclasses import dataclass
from typing import Union


@dataclass(frozen=True)
class TMotorData:
    position: float
    velocityERPM: float = 0.0
    mode: int = 0
    useBrake: int = 0


@dataclass(frozen=True)
class MaxonData:
    position: float
    mode: int = 1
    kp: int = 0
    kd: int = 0


CanMotorCommand = Union[TMotorData, MaxonData]
