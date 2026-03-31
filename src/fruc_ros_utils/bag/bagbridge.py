#!/usr/bin/env python3
"""Unified dispatcher CLI for ROS1/ROS2 bag utilities.

Usage:
  bagutils ros1 <ros1utils args...>
  bagutils ros2 <ros2utils args...>

Legacy convenience:
  bagutils <ros1 command ...>      # forwarded to ros1utils
  bagutils <ros2 command ...>      # forwarded to ros2utils when command matches
"""

from __future__ import annotations

import importlib
import sys
from typing import Iterable


ROS2_COMMANDS = {
    "info",
    "list_topics",
    "duration",
    "extract_json",
    "convert_to_ros1",
    "convert_folder",
}


def _dispatch(module_name: str, forwarded_argv: Iterable[str]) -> int:
    module = importlib.import_module(module_name)
    main_fn = getattr(module, "main", None)
    if main_fn is None:
        raise RuntimeError(f"{module_name} has no main() entrypoint")

    argv_backup = sys.argv[:]
    try:
        sys.argv = [module_name.rsplit(".", 1)[-1], *list(forwarded_argv)]
        main_fn()
        return 0
    finally:
        sys.argv = argv_backup


def main() -> int:
    argv = sys.argv[1:]
    if not argv:
        print(
            "Usage:\n"
            "  bagutils ros1 <ros1utils args...>\n"
            "  bagutils ros2 <ros2utils args...>\n"
            "  bagutils <legacy ros1/ros2 command ...>"
        )
        return 2

    head = argv[0]
    if head in ("ros1", "ros1utils"):
        return _dispatch("fruc_ros_utils.bag.ros1utils", argv[1:])
    if head in ("ros2", "ros2utils"):
        return _dispatch("fruc_ros_utils.bag.ros2utils", argv[1:])
    if head in ROS2_COMMANDS:
        return _dispatch("fruc_ros_utils.bag.ros2utils", argv)
    return _dispatch("fruc_ros_utils.bag.ros1utils", argv)


if __name__ == "__main__":
    raise SystemExit(main())
