#!/usr/bin/env python3
"""Simple ROS1→MCAP converter using subprocess and rosbags-convert."""

import argparse
import sys
import subprocess
import shutil
import tempfile
from pathlib import Path
from typing import List, Optional
import logging

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)

from _deps import ensure_rosbags as _ensure_rosbags


def convert_ros1_to_mcap(bag_path: Path, mcap_path: Path) -> bool:
    """Convert ROS1 bag → rosbag2 directory → MCAP file.

    Uses rosbags-convert CLI tool with temporary directory for intermediate conversion.
    """
    if not bag_path.exists():
        logger.error(f"Input not found: {bag_path}")
        return False

    try:
        logger.info(f"Converting: {bag_path.name} → {mcap_path.name}")

        # Create temporary directory for rosbag2 intermediate format
        with tempfile.TemporaryDirectory(prefix=".ros1_convert_") as tmpdir:
            tmpdir_path = Path(tmpdir)
            temp_bag2_dir = tmpdir_path / "temp_bag2"

            logger.debug(f"  Step 1: ROS1 → rosbag2 directory format")
            import os
            env = os.environ.copy()
            env["PATH"] = f"{Path.home() / '.local' / 'bin'}:{env.get('PATH', '')}"

            # Step 1: ROS1 → rosbag2 directory
            cmd1 = ["rosbags-convert", "--dst", str(temp_bag2_dir), str(bag_path)]
            result = subprocess.run(cmd1, capture_output=True, text=True, env=env, timeout=600)
            if result.returncode != 0:
                logger.error(f"  rosbags-convert step 1 failed")
                if result.stderr:
                    logger.error(f"    Error: {result.stderr}")
                return False

            logger.debug(f"  Step 2: rosbag2 directory → MCAP file")

            # Step 2: rosbag2 directory → MCAP file
            # Need to use a temporary MCAP name since rosbags-convert won't overwrite
            temp_mcap = tmpdir_path / "temp.mcap"
            cmd2 = ["rosbags-convert", "--dst", str(temp_mcap), str(temp_bag2_dir)]
            result = subprocess.run(cmd2, capture_output=True, text=True, env=env, timeout=600)
            if result.returncode != 0:
                logger.error(f"  rosbags-convert step 2 failed")
                if result.stderr:
                    logger.error(f"    Error: {result.stderr}")
                return False

            # Copy temp MCAP to final location
            if temp_mcap.exists():
                shutil.copy2(temp_mcap, mcap_path)
                logger.info(f"  ✓ Successfully created {mcap_path.name}")
                return True
            else:
                logger.error(f"  rosbags-convert did not create output file")
                return False

    except subprocess.TimeoutExpired:
        logger.error(f"  Conversion timed out")
        return False
    except Exception as e:
        logger.error(f"  Conversion failed: {e}", exc_info=True)
        return False


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert ROS1 bag chunks to MCAP files"
    )
    parser.add_argument("--bags-dir", type=str, default="/bags", help="Input bags directory")
    parser.add_argument("--output-dir", type=str, default="/bags_out", help="Output MCAP directory")
    parser.add_argument("--chunk-indices", type=int, nargs="+", default=[5, 6, 7, 8, 9],
                        help="Chunk indices to convert")
    parser.add_argument("--name-prefix", type=str, default="2026_03_27_17_02_24__icnf-curt",
                        help="Filename prefix")
    parser.add_argument("--verbose", "-v", action="store_true")

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
    failed = []
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
        else:
            failed.append(idx)
        logger.info("")

    logger.info("=" * 70)
    logger.info(f"Complete: {success}/{len(args.chunk_indices)} chunks")
    if failed:
        logger.warning(f"Failed chunks: {failed}")
    logger.info("=" * 70)

    return 0 if success > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
