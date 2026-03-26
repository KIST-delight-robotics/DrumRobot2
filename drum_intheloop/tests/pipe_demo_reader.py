#!/usr/bin/env python3

import json
import os
from pathlib import Path


PIPE_PATH = Path("/tmp/drum_pipe_demo")


def ensure_pipe_exists() -> None:
    if not PIPE_PATH.exists():
        os.mkfifo(PIPE_PATH)


def main() -> int:
    ensure_pipe_exists()
    print(f"Listening on {PIPE_PATH} ...")

    with PIPE_PATH.open("r", encoding="utf-8") as pipe:
        for raw_line in pipe:
            line = raw_line.strip()
            if not line:
                continue

            payload = json.loads(line)
            print(f"Raw line: {line}")
            print(
                "Parsed -> "
                f"kind={payload['kind']}, "
                f"motor={payload['motor']}, "
                f"position={payload['position']}, "
                f"mode={payload['mode']}"
            )

    print("Reader finished.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
