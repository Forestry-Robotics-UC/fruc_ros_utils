#!/usr/bin/env python3
"""Convert ROS1 bags to MCAP files using rosbags library."""

import argparse
import sys
import subprocess
from pathlib import Path
from typing import List, Optional
import logging

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

from _deps import ensure_rosbags as _ensure_rosbags


def convert_ros1_to_mcap(bag_path: Path, output_path: Path) -> bool:
    """Convert ROS1 bag to MCAP file using rosbags command-line tool.

    Args:
        bag_path: Input ROS1 .bag file
        output_path: Output MCAP file

    Returns:
        True if successful
    """
    if not bag_path.exists():
        logger.error(f"Input not found: {bag_path}")
        return False

    try:
        logger.info(f"Converting: {bag_path.name} → {output_path.name}")

        # Create temporary rosbag2 directory
        temp_bag2_dir = output_path.parent / f".tmp__{output_path.stem}"

        # Step 1: Convert ROS1 → rosbag2 (directory format)
        logger.debug(f"  Step 1: ROS1 → rosbag2 directory")
        cmd1 = ["rosbags-convert", "--dst", str(temp_bag2_dir), str(bag_path)]
        result = subprocess.run(cmd1, capture_output=True, text=True, env=_get_rosbags_env())
        if result.returncode != 0:
            logger.error(f"  rosbags-convert failed: {result.stderr}")
            return False

        # Step 2: Convert rosbag2 directory → MCAP file
        logger.debug(f"  Step 2: rosbag2 → MCAP file")
        cmd2 = ["rosbags-convert", "--dst", str(output_path), str(temp_bag2_dir)]
        result = subprocess.run(cmd2, capture_output=True, text=True, env=_get_rosbags_env())
        if result.returncode != 0:
            logger.error(f"  rosbags-convert failed: {result.stderr}")
            return False

        # Cleanup temp directory
        import shutil
        shutil.rmtree(temp_bag2_dir, ignore_errors=True)

        logger.info(f"  ✓ Successfully created {output_path.name}")
        return True

    except Exception as e:
        logger.error(f"Conversion failed: {e}", exc_info=True)
        return False


def _get_rosbags_env():
    """Get environment with rosbags-convert in PATH."""
    import os
    env = os.environ.copy()
    local_bin = Path.home() / ".local" / "bin"
    env["PATH"] = f"{local_bin}:{env.get('PATH', '')}"
    return env


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert ROS1 bags to MCAP files"
    )

    parser.add_argument(
        "--bags-dir",
        type=str,
        default="/bags",
        help="Input bags directory",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/bags_out",
        help="Output MCAP directory",
    )
    parser.add_argument(
        "--chunk-indices",
        type=int,
        nargs="+",
        default=[5, 6, 7, 8, 9],
        help="Chunk indices to convert",
    )
    parser.add_argument(
        "--name-prefix",
        type=str,
        default="2026_03_27_17_02_24__icnf-curt",
        help="Filename prefix",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
    )

    args = parser.parse_args(argv)

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if not _ensure_rosbags():
        return 1

    bags_dir = Path(args.bags_dir)
    output_dir = Path(args.output_dir)

    if not bags_dir.exists():
        logger.error(f"Bags dir not found: {bags_dir}")
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("=" * 70)
    logger.info("ROS1 Chunks → MCAP Conversion")
    logger.info("=" * 70)

    success = 0
    for idx in args.chunk_indices:
        bag_file = bags_dir / f"{args.name_prefix}__ros1_chunk_{idx:03d}.bag"
        mcap_file = output_dir / f"{args.name_prefix}__{idx}.mcap"

        if not bag_file.exists():
            logger.warning(f"Chunk {idx}: Not found")
            continue

        if mcap_file.exists():
            logger.info(f"Chunk {idx}: Already exists (skipping)")
            continue

        if convert_ros1_to_mcap(bag_file, mcap_file):
            success += 1
        logger.info("")

    logger.info("=" * 70)
    logger.info(f"Complete: {success}/{len(args.chunk_indices)} chunks converted")
    logger.info("=" * 70)

    return 0 if success > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
