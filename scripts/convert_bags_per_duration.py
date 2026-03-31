#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility wrapper for duration-based ROS 2 -> ROS 1 conversion.

This script now delegates all bag splitting and conversion to
``fruc_ros_utils.bag.ros2utils.Ros2BagUtils`` so there is only one implementation of the
split-and-convert logic in the repo.
"""

from __future__ import annotations

import argparse
import logging
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Iterable, List


logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


def _ros2_bag_utils_cls():
    """Import Ros2BagUtils lazily so `--help` works without a ROS 2 runtime."""
    from fruc_ros_utils.bag.ros2utils import Ros2BagUtils

    return Ros2BagUtils


def _discover_bags(input_folder: Path, pattern: str) -> List[Path]:
    """Discover ROS 2 bag files under ``input_folder`` using a glob pattern."""
    return sorted(
        path
        for path in input_folder.glob(pattern)
        if path.is_file() and path.suffix.lower() in (".mcap", ".db3")
    )


def _split_duration_arg(duration_s: int, bag_duration_s: float, skip_split_if_under: int) -> str | None:
    """Return ``<N>s`` split arg when splitting should be enabled, else ``None``."""
    if skip_split_if_under > 0 and bag_duration_s < float(skip_split_if_under):
        return None
    if bag_duration_s <= float(duration_s):
        return None
    return f"{int(duration_s)}s"


def _convert_one_bag(
    bag_path: Path,
    output_folder: Path,
    *,
    duration_s: int,
    src_typestore: str,
    dst_typestore: str,
    preserve_lidar_fields: bool,
    lidar_topic: str,
    validate: bool,
    skip_split_if_under: int,
) -> int:
    """Convert one ROS 2 bag to ROS 1, optionally enabling duration splitting."""
    utils = _ros2_bag_utils_cls()()
    total_duration = utils.bag_duration(str(bag_path))
    split_duration = _split_duration_arg(duration_s, total_duration, skip_split_if_under)
    out_path = output_folder / f"{bag_path.stem}_ros1.bag"

    logger.info("Processing %s", bag_path.name)
    logger.info("  Total bag duration: %.1fs", total_duration)
    if split_duration is None:
        logger.info("  Converting without split")
    else:
        logger.info("  Converting with built-in split threshold %s", split_duration)

    result = utils.convert_to_ros1(
        str(bag_path),
        out_path=str(out_path),
        src_typestore=src_typestore,
        dst_typestore=dst_typestore,
        validate=validate,
        preserve_lidar_fields=preserve_lidar_fields,
        lidar_topic=lidar_topic,
        exclude_topics=["/ouster/metadata", "/ouster/lidar_packets"],
        split_duration=split_duration,
    )

    produced = result if isinstance(result, list) else [result]
    logger.info("  Wrote %d output bag(s) for %s", len(produced), bag_path.name)
    return len(produced)


def _convert_many_bags(
    bag_paths: Iterable[Path],
    output_folder: Path,
    *,
    duration_s: int,
    src_typestore: str,
    dst_typestore: str,
    preserve_lidar_fields: bool,
    lidar_topic: str,
    validate: bool,
    skip_split_if_under: int,
    parallel_workers: int,
) -> int:
    """Convert many bags and return the total number of produced ROS 1 bag files."""
    bags = list(bag_paths)
    if not bags:
        return 0

    if parallel_workers <= 1 or len(bags) == 1:
        return sum(
            _convert_one_bag(
                bag_path,
                output_folder,
                duration_s=duration_s,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                preserve_lidar_fields=preserve_lidar_fields,
                lidar_topic=lidar_topic,
                validate=validate,
                skip_split_if_under=skip_split_if_under,
            )
            for bag_path in bags
        )

    logger.info("Processing %d bag(s) with %d parallel worker(s)", len(bags), parallel_workers)
    converted = 0
    with ThreadPoolExecutor(max_workers=parallel_workers) as executor:
        futures = {
            executor.submit(
                _convert_one_bag,
                bag_path,
                output_folder,
                duration_s=duration_s,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                preserve_lidar_fields=preserve_lidar_fields,
                lidar_topic=lidar_topic,
                validate=validate,
                skip_split_if_under=skip_split_if_under,
            ): bag_path
            for bag_path in bags
        }
        for future in as_completed(futures):
            bag_path = futures[future]
            try:
                converted += future.result()
            except Exception as exc:
                logger.error("Failed to convert %s: %s", bag_path, exc)
    return converted


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert ROS 2 bags to ROS 1 with duration-based chunking",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --input /path/to/input.mcap --output-folder /path/to/ros1_bags --duration 300
  %(prog)s --input-folder /path/to/ros2_bags --output-folder /path/to/ros1_bags --duration 300
  %(prog)s --input-folder /path/to/ros2_bags --output-folder /path/to/ros1_bags --duration 300 --parallel-chunks 4
        """,
    )

    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--input", type=str, help="Path to single ROS 2 bag (.mcap or .db3)")
    input_group.add_argument(
        "--input-folder",
        type=str,
        help="Path to folder containing ROS 2 bags (.mcap and .db3)",
    )

    parser.add_argument("--output-folder", type=str, required=True, help="Output folder for ROS 1 bags")
    parser.add_argument("--duration", type=int, default=300, help="Target split duration in seconds")
    parser.add_argument("--src-typestore", type=str, default="ros2_jazzy")
    parser.add_argument("--dst-typestore", type=str, default="ros1_noetic")
    parser.add_argument(
        "--preserve-lidar-fields",
        action="store_true",
        help="Restore custom LiDAR fields (ring, reflectivity) from MCAP",
    )
    parser.add_argument(
        "--lidar-topic",
        type=str,
        default="/ouster/points/corrected",
        help="LiDAR topic to restore fields for",
    )
    parser.add_argument("--no-validate", action="store_true", help="Skip post-conversion validation")
    parser.add_argument(
        "--parallel-chunks",
        type=int,
        default=1,
        help="Number of parallel workers when converting multiple input bags",
    )
    parser.add_argument(
        "--skip-split-if-under",
        type=int,
        default=0,
        help="Skip split for bags shorter than this many seconds",
    )
    parser.add_argument(
        "--pattern",
        type=str,
        default="*",
        help="Glob pattern for folder discovery (default: '*', filtered to .mcap/.db3)",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose logging")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    output_folder = Path(args.output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    if args.input:
        bag_paths = [Path(args.input)]
        if not bag_paths[0].exists():
            logger.error("Input bag not found: %s", bag_paths[0])
            return 1
        if bag_paths[0].suffix.lower() not in (".mcap", ".db3"):
            logger.error("Input bag must be .mcap or .db3: %s", bag_paths[0])
            return 1
    else:
        input_folder = Path(args.input_folder)
        if not input_folder.exists():
            logger.error("Input folder not found: %s", input_folder)
            return 1
        bag_paths = _discover_bags(input_folder, args.pattern)
        if not bag_paths:
            logger.error("No ROS 2 bags found in %s with pattern %s", input_folder, args.pattern)
            return 1

    logger.info("Found %d bag(s) to process", len(bag_paths))
    converted = _convert_many_bags(
        bag_paths,
        output_folder,
        duration_s=args.duration,
        src_typestore=args.src_typestore,
        dst_typestore=args.dst_typestore,
        preserve_lidar_fields=args.preserve_lidar_fields,
        lidar_topic=args.lidar_topic,
        validate=not args.no_validate,
        skip_split_if_under=args.skip_split_if_under,
        parallel_workers=max(1, int(args.parallel_chunks)),
    )

    logger.info("%s", "=" * 60)
    logger.info("Total output bag files produced: %d", converted)
    logger.info("Output folder: %s", output_folder)
    logger.info("%s", "=" * 60)
    return 0 if converted > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
