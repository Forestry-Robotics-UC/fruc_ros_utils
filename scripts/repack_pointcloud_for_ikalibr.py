#!/usr/bin/env python3
"""Compatibility wrapper for the packaged iKalibr point-cloud repack helper."""

from __future__ import annotations

import sys
from pathlib import Path

try:
    from bag.repack_pointcloud_for_ikalibr import main
except ImportError:
    repo_root = Path(__file__).resolve().parents[1]
    src_root = repo_root / "src"
    if src_root.exists():
        sys.path.insert(0, str(src_root))
    from bag.repack_pointcloud_for_ikalibr import main


if __name__ == "__main__":
    raise SystemExit(main())
