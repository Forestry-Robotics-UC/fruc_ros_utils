#!/usr/bin/env python3
"""Entrypoint for packet converter image without shell scripts."""

from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
    script_path = "/app/scripts/convert_bag_packets.py"
    if not os.path.isfile(script_path):
        print(f"ERROR: {script_path} not found", file=sys.stderr)
        return 1

    args = sys.argv[1:] or ["--help"]
    setup_cmd = (
        "set -euo pipefail; "
        "set +u; "
        "if [ -f /docker_ws/install/setup.bash ]; then source /docker_ws/install/setup.bash || true; fi; "
        "set -u; "
        "if [ -f /app/.venv/bin/activate ]; then source /app/.venv/bin/activate; fi; "
        "exec python /app/scripts/convert_bag_packets.py \"$@\""
    )
    return subprocess.call(["/bin/bash", "-lc", setup_cmd, "--", *args])


if __name__ == "__main__":
    raise SystemExit(main())
