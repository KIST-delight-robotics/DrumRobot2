from typing import Dict, List, Optional

from .mapping import MAXON_SPEC


PRE_OPERATIONAL = "pre_operational"
OPERATIONAL = "operational"
STOPPED = "stopped"


_KIND_TO_STATE: Dict[str, str] = {
    "nmt_start": OPERATIONAL,
    "nmt_stop": STOPPED,
    "nmt_preop": PRE_OPERATIONAL,
    "nmt_reset": PRE_OPERATIONAL,
}


class NmtState:
    def __init__(self) -> None:
        self._state: Dict[str, str] = {motor: PRE_OPERATIONAL for motor in MAXON_SPEC}

    def is_operational(self, motor: str) -> bool:
        return self._state.get(motor) == OPERATIONAL

    def transition(self, kind: str, motor: Optional[str]) -> None:
        next_state = _KIND_TO_STATE.get(kind)
        if next_state is None:
            return

        targets: List[str] = [motor] if motor else list(self._state.keys())
        for target in targets:
            if target in self._state:
                self._state[target] = next_state
