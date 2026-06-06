"""Shared dependency bootstrap helpers for standalone converter scripts."""

import subprocess
import sys
import logging

logger = logging.getLogger(__name__)


def ensure_rosbags() -> bool:
    """Ensure rosbags (with MCAP support) and mcap packages are available.

    Attempts to install via pip if missing. Returns True when available.
    """
    missing = []
    try:
        import rosbags  # noqa: F401
        from rosbags.rosbag1 import Reader as _R1  # noqa: F401
    except ImportError:
        missing.append("rosbags")

    try:
        from mcap.writer import Writer  # noqa: F401
    except ImportError:
        missing.append("mcap")

    if missing:
        logger.info("Installing missing dependencies: %s", ", ".join(missing))
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "--quiet", "rosbags[mcap]", "mcap"],
        )

    return True
