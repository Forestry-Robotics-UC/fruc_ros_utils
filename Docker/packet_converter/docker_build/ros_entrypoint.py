#!/usr/bin/env python3
"""ROS runtime entrypoint for rqt_bag image without shell scripts."""

from __future__ import annotations

import subprocess
import sys


def main() -> int:
    args = sys.argv[1:] or ["rqt_bag"]
    cmd = "source /opt/ros/$ROS_DISTRO/setup.bash && exec \"$@\""
    return subprocess.call(["/bin/bash", "-lc", cmd, "--", *args])


if __name__ == "__main__":
    raise SystemExit(main())
