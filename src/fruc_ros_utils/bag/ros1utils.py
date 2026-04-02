#!/usr/bin/env python3
"""
Compatibility wrapper: expose existing ROS1 bag utilities under
the `ros1utils` entry point, and add a convenience command to
export CameraInfo into an iKalibr intrinsics YAML.

This file intentionally re-exports symbols from `bag.bagutils`
so existing imports continue to work while providing a new
CLI entrypoint name.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys

# Re-export everything from the original bagutils implementation
from fruc_ros_utils.bag.bagutils import *  # noqa: F401,F403
from fruc_ros_utils.bag import bagutils as _bagutils
from fruc_ros_utils.utils.logging_utils import get_logger

try:
    import argcomplete
except Exception:
    argcomplete = None

try:
    from fruc_ros_utils.bag import export_camera_info as _export_script
except Exception:
    _export_script = None

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

    if "export_camera_info" not in subparsers.choices:
        sp = subparsers.add_parser(
            "export_camera_info",
            help="Export CameraInfo from a ROS1 bag to iKalibr YAML",
        )
        sp.add_argument("bagfile", help="Input ROS1 bag path")
        sp.add_argument("camera_info_topic", help="CameraInfo topic in the input bag")
        sp.add_argument("out_yaml", help="Output YAML file path")

    def _add_repack_parser(name: str) -> None:
        if name in subparsers.choices:
            return
        sp = subparsers.add_parser(
            name,
            help="Repack PointCloud2 fields for iKalibr",
        )
        sp.add_argument("in_bag", nargs="?", help="Input ROS1 .bag (legacy positional form)")
        sp.add_argument("out_bag", nargs="?", help="Output ROS1 .bag (legacy positional form)")
        sp.add_argument(
            "topic",
            nargs="?",
            default="/ouster/points/corrected",
            help="PointCloud2 topic to repack (legacy positional form)",
        )
        sp.add_argument("--in", dest="in_bag_flag", help="Input ROS1 .bag")
        sp.add_argument("--out", dest="out_bag_flag", help="Output ROS1 .bag")
        sp.add_argument(
            "--topic",
            dest="topic_flag",
            default="/ouster/points/corrected",
            help="PointCloud2 topic to repack",
        )
        sp.add_argument("--overwrite", action="store_true", help="Overwrite output if it exists")
        sp.add_argument(
            "--ring-dtype",
            dest="ring_dtype",
            choices=("auto", "uint8", "uint16"),
            default="auto",
            help="Force ring field datatype (default: auto)",
        )

    _add_repack_parser("repack_pointcloud")
    _add_repack_parser("repack_pointcloud_for_ikalibr")

    if "crop_pointcloud_fov" not in subparsers.choices:
        sp = subparsers.add_parser(
            "crop_pointcloud_fov",
            help="Crop a ROS1 PointCloud2 bag topic by horizontal FOV",
        )
        sp.add_argument("--in", dest="in_bag", required=True, help="Input ROS1 .bag")
        sp.add_argument("--out", dest="out_bag", required=True, help="Output ROS1 .bag")
        sp.add_argument(
            "--topic",
            default="/ouster/points",
            help="PointCloud2 topic to crop",
        )
        sp.add_argument(
            "--fov-deg",
            type=float,
            default=120.0,
            help="Horizontal field of view to keep",
        )
        sp.add_argument(
            "--center-deg",
            type=float,
            default=0.0,
            help="Center azimuth in degrees",
        )
        sp.add_argument(
            "--overwrite",
            action="store_true",
            help="Overwrite output if it exists",
        )

    if argcomplete:
        argcomplete.autocomplete(parser)

    return parser


def _run_completion(argv: list[str]) -> None:
    if "_ARGCOMPLETE" not in os.environ:
        return
    parser = _build_completion_parser()
    parser.parse_known_args(argv)


def export_camera_info(bagfile: str, topic: str, outpath: str) -> None:
    """Export a CameraInfo message from a ROS1 bag into an iKalibr YAML.

    This calls the existing `bag/export_camera_info.py` script's `main`
    function programmatically to avoid spawning subprocesses.
    """
    if _export_script is None:
        raise RuntimeError("export_camera_info helper not available (missing module)")

    # Call the script's main with controlled argv
    logger.debug(
        "ros1utils export_camera_info bagfile=%s topic=%s outpath=%s",
        bagfile,
        topic,
        outpath,
    )
    argv_backup = sys.argv
    try:
        sys.argv = ["export_camera_info.py", bagfile, topic, outpath]
        _export_script.main()
    finally:
        sys.argv = argv_backup


def repack_pointcloud_for_ikalibr(
    in_bag: str,
    out_bag: str,
    topic: str = "/ouster/points/corrected",
    overwrite: bool = False,
    ring_dtype: str = "auto",
) -> None:
    """Run the `scripts/repack_pointcloud_for_ikalibr.py` helper on the given bags.

    This loads and executes the script in-process by setting `sys.argv` so the
    script can be reused without spawning a subprocess.
    """
    from fruc_ros_utils.bag import repack_pointcloud_for_ikalibr as repack_tool

    logger.debug(
        "ros1utils repack_pointcloud_for_ikalibr in_bag=%s out_bag=%s topic=%s overwrite=%s",
        in_bag,
        out_bag,
        topic,
        overwrite,
    )
    argv = ["--in", in_bag, "--out", out_bag, "--topic", topic]
    if overwrite:
        argv.append("--overwrite")
    argv.extend(["--ring-dtype", ring_dtype])
    repack_tool.main(argv)


def crop_pointcloud_fov(argv: list[str]) -> None:
    """Run the `scripts/crop_pointcloud_fov.py` helper with the provided argv."""
    from fruc_ros_utils.bag import crop_pointcloud_fov as crop_tool

    logger.debug("ros1utils crop_pointcloud_fov argv=%s", argv)
    crop_tool.main(list(argv))


def main() -> None:
    """Alias entrypoint so package scripts can point to bag.ros1utils:main"""
    # Reuse original bag.bagutils main if present
    # Provide a thin wrapper: intercept the `export_camera_info` subcommand
    # to call the helper directly; otherwise delegate to the original main.
    argv = sys.argv[1:]
    _reconfigure_logger_from_argv(argv)
    _run_completion(argv)
    logger.debug("ros1utils argv=%s", sys.argv)
    if len(argv) >= 1 and argv[0] == "export_camera_info":
        # Expected usage: ros1utils export_camera_info <bagfile> <camera_info_topic> <out_yaml>
        if _export_script is None:
            raise RuntimeError("export_camera_info helper not available")
        # delegate to the helper script's main
        logger.debug(
            "Delegating ros1utils export_camera_info to "
            "bag.export_camera_info.main"
        )
        return _export_script.main()
    if len(argv) >= 1 and argv[0] in ("repack_pointcloud", "repack_pointcloud_for_ikalibr"):
        # Backward-compatible positional usage:
        #   ros1utils repack_pointcloud <in.bag> <out.bag> [topic]
        # Full-flag usage (preferred):
        #   ros1utils repack_pointcloud --in <in.bag> --out <out.bag> [--topic ...] [--overwrite] [--ring-dtype ...]
        if len(argv) >= 2 and argv[1].startswith("-"):
            from fruc_ros_utils.bag import repack_pointcloud_for_ikalibr as repack_tool

            logger.debug("Handling ros1utils repack_pointcloud flag-style wrapper command")
            repack_tool.main(argv[1:])
            return

        if len(argv) < 3:
            raise RuntimeError(
                "Usage: ros1utils repack_pointcloud <in.bag> <out.bag> [topic]"
            )
        in_bag = argv[1]
        out_bag = argv[2]
        topic = argv[3] if len(argv) >= 4 else "/ouster/points/corrected"
        logger.debug("Handling ros1utils repack_pointcloud positional wrapper command")
        repack_pointcloud_for_ikalibr(in_bag, out_bag, topic=topic)
        return
    if len(argv) >= 1 and argv[0] == "crop_pointcloud_fov":
        logger.debug("Handling ros1utils crop_pointcloud_fov wrapper command")
        crop_pointcloud_fov(argv[1:])
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
