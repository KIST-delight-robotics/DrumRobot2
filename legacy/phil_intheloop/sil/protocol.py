import json
from typing import Dict, List, Tuple


def split_lines(buffer: str) -> Tuple[List[str], str]:
    if "\n" not in buffer:
        return [], buffer

    lines = []
    while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)
        line = line.strip()
        if line:
            lines.append(line)
    return lines, buffer


def unpack_payload(payload: str) -> List[str]:
    stripped = (payload or "").strip()
    if not stripped:
        return []

    if stripped.startswith("{"):
        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            return [stripped]

        commands = parsed.get("commands")
        if isinstance(commands, list):
            return [str(command).strip() for command in commands if str(command).strip()]

    return [stripped]


def serialize_state(state: Dict[str, object]) -> str:
    return json.dumps(state, ensure_ascii=False)
