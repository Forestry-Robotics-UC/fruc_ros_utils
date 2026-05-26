"""Shared ROS1 CLI extensions layered on top of bagutils."""

from __future__ import annotations

import argparse
import logging
import sys


def add_ros1_extension_subparsers(subparsers) -> None:
    """Register ros1utils-only convenience subcommands."""
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


def handle_ros1_extension_command(argv: list[str], logger: logging.Logger) -> bool:
    """Run a ros1utils-only convenience command when ``argv`` matches one."""
    if not argv:
        return False

    if argv[0] == "crop_pointcloud_fov":
        from fruc_ros_utils.bag import crop_pointcloud_fov as crop_tool

        logger.debug("Handling ros1utils crop_pointcloud_fov wrapper command")
        crop_tool.main(argv[1:])
        return True

    return False
