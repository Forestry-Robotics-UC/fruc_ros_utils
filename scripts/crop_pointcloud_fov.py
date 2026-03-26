#!/usr/bin/env python3
"""Compatibility wrapper for the packaged point-cloud FOV crop helper."""

from __future__ import annotations

import sys
from pathlib import Path

try:
    from bag.crop_pointcloud_fov import main
except ImportError:
    repo_root = Path(__file__).resolve().parents[1]
    src_root = repo_root / "src"
    if src_root.exists():
        sys.path.insert(0, str(src_root))
    from bag.crop_pointcloud_fov import main


if __name__ == "__main__":
    raise SystemExit(main())
