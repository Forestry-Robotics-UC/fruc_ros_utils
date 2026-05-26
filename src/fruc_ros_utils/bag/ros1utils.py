#!/usr/bin/env python3
"""Compatibility wrapper exposing ROS 1 bag utilities under ``ros1utils``."""
from __future__ import annotations

import argparse
import logging
import os
import sys

from fruc_ros_utils.bag.bagutils import *  # noqa: F401,F403
from fruc_ros_utils.bag import bagutils as _bagutils
from fruc_ros_utils.bag.ros1_cli_extensions import (
    add_ros1_extension_subparsers,
    handle_ros1_extension_command,
)
from fruc_ros_utils.utils.logging_utils import get_logger

try:
    import argcomplete
except Exception:
    argcomplete = None

_default_level = os.environ.get("BAGUTILS_LOG_LEVEL", "INFO").upper()
_default_file = os.environ.get("BAGUTILS_LOG_FILE", None)
logger = get_logger("Ros1utils", level=_default_level, log_file=_default_file)


def _reconfigure_logger_from_argv(argv: list[str]) -> None:
    global logger
    level = _default_level
    log_file = _default_file
    for index, arg in enumerate(argv):
        if arg == "--log-level" and index + 1 < len(argv):
            level = argv[index + 1]
        elif arg.startswith("--log-level="):
            level = arg.split("=", 1)[1]
        elif arg == "--log-file" and index + 1 < len(argv):
            log_file = argv[index + 1]
        elif arg.startswith("--log-file="):
            log_file = arg.split("=", 1)[1]
    logger = get_logger("Ros1utils", level=str(level).upper(), log_file=log_file)


def _find_subparsers_action(
    parser: argparse.ArgumentParser,
) -> argparse._SubParsersAction:
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            return action
    raise RuntimeError("Could not find subparser action in ros1utils parser")


def _build_completion_parser() -> argparse.ArgumentParser:
    parser = _bagutils.build_parser(enable_shell_completion=False)
    subparsers = _find_subparsers_action(parser)
    add_ros1_extension_subparsers(subparsers)

    if argcomplete:
        argcomplete.autocomplete(parser)

    return parser


def _run_completion(argv: list[str]) -> None:
    if "_ARGCOMPLETE" not in os.environ:
        return
    parser = _build_completion_parser()
    parser.parse_known_args(argv)


def main() -> None:
    """Alias entrypoint so package scripts can point to bag.ros1utils:main"""
    argv = sys.argv[1:]
    _reconfigure_logger_from_argv(argv)
    _run_completion(argv)
    logger.debug("ros1utils argv=%s", sys.argv)
    if handle_ros1_extension_command(argv, logger):
        return

    try:
        from fruc_ros_utils.bag.bagutils import main as _main
    except Exception as e:
        logging.error("Failed to locate bag.bagutils.main: %s", e)
        raise
    logger.debug("Delegating ros1utils command to bag.bagutils.main")
    return _main()


if __name__ == "__main__":
    main()
