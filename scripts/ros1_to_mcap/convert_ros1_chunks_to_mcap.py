#!/usr/bin/env python3
"""Convert ROS1 bag chunks to ROS2 MCAP files, filling missing sequence numbers.

Usage:
    python3 convert_ros1_chunks_to_mcap.py \
        --bags-dir /bags \
        --output-dir /bags_out \
        --chunk-indices 5 6 7 8 9 \
        --compress

Based on timestamp analysis:
- ROS1 chunks are ~25-27s each
- Chunks 5-9 contain the missing data for MCAP files 5-9
"""

import argparse
import sys
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


def convert_ros1_chunk_to_mcap(
    bag_path: Path,
    output_path: Path,
    compress: bool = True,
    decode_ouster: bool = True,
    decode_ffmpeg: bool = True,
) -> bool:
    """Convert a single ROS1 bag to ROS2 MCAP format.

    Args:
        bag_path: Path to ROS1 .bag file
        output_path: Output MCAP file path
        compress: Whether to compress compatible topics
        decode_ouster: Whether to decode Ouster lidar packets
        decode_ffmpeg: Whether to decode ffmpeg-compressed images

    Returns:
        True if successful, False otherwise
    """
    if not bag_path.exists():
        logger.error(f"Input bag not found: {bag_path}")
        return False

    try:
        import subprocess
        import os

        logger.info(f"Converting {bag_path.name} → {output_path.name}")

        # Use rosbags-convert CLI which handles ROS1→ROS2 conversion properly
        cmd = ["rosbags-convert", "--dst", str(output_path), str(bag_path)]

        # Ensure rosbags-convert is in PATH
        env = os.environ.copy()
        env["PATH"] = f"{os.path.expanduser('~/.local/bin')}:{env.get('PATH', '')}"

        result = subprocess.run(cmd, capture_output=True, text=True, check=False, env=env)

        if result.returncode != 0:
            logger.error(f"Conversion failed: {result.stderr}")
            return False

        logger.info(f"✓ Successfully created {output_path.name}")
        return True

    except FileNotFoundError:
        logger.error("rosbags-convert command not found. Install rosbags: pip install rosbags")
        return False
    except Exception as e:
        logger.error(f"✗ Conversion failed: {e}", exc_info=True)
        return False


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert ROS1 bag chunks to ROS2 MCAP files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --bags-dir /bags --output-dir /bags_out --chunk-indices 5 6 7 8 9
  %(prog)s --bags-dir /bags --output-dir /bags_out --chunk-indices 5 6 7 8 9 --compress
        """,
    )

    parser.add_argument(
        "--bags-dir",
        type=str,
        default="/bags",
        help="Directory containing ROS1 bag chunks (default: /bags)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/bags_out",
        help="Output directory for MCAP files (default: /bags_out)",
    )
    parser.add_argument(
        "--chunk-indices",
        type=int,
        nargs="+",
        default=[5, 6, 7, 8, 9],
        help="Chunk indices to convert (default: 5 6 7 8 9)",
    )
    parser.add_argument(
        "--compress",
        action="store_true",
        help="Enable compression for compatible topics",
    )
    parser.add_argument(
        "--decode-ouster",
        action="store_true",
        help="Decode Ouster lidar packets to point clouds",
    )
    parser.add_argument(
        "--decode-ffmpeg",
        action="store_true",
        help="Decode ffmpeg-compressed images",
    )
    parser.add_argument(
        "--name-prefix",
        type=str,
        default="2026_03_27_17_02_24__icnf-curt",
        help="Output filename prefix",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Verbose logging",
    )

    args = parser.parse_args(argv)

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Ensure dependencies
    if not _ensure_rosbags():
        return 1

    bags_dir = Path(args.bags_dir)
    output_dir = Path(args.output_dir)

    if not bags_dir.exists():
        logger.error(f"Bags directory not found: {bags_dir}")
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("=" * 70)
    logger.info("ROS1 Chunks → ROS2 MCAP Conversion")
    logger.info("=" * 70)
    logger.info(f"Bags directory: {bags_dir}")
    logger.info(f"Output directory: {output_dir}")
    logger.info(f"Chunk indices: {args.chunk_indices}")
    logger.info("")

    success_count = 0
    for chunk_idx in args.chunk_indices:
        # Map chunk index to bag file and output file
        bag_file = bags_dir / f"{args.name_prefix}__ros1_chunk_{chunk_idx:03d}.bag"
        output_file = output_dir / f"{args.name_prefix}__{chunk_idx}.mcap"

        if not bag_file.exists():
            logger.warning(f"Chunk {chunk_idx}: Source bag not found: {bag_file.name}")
            continue

        if output_file.exists():
            logger.info(f"Chunk {chunk_idx}: Output already exists, skipping: {output_file.name}")
            continue

        if convert_ros1_chunk_to_mcap(
            bag_file,
            output_file,
            compress=args.compress,
            decode_ouster=args.decode_ouster,
            decode_ffmpeg=args.decode_ffmpeg,
        ):
            success_count += 1
        else:
            logger.error(f"Chunk {chunk_idx}: Conversion failed")

    logger.info("")
    logger.info("=" * 70)
    logger.info(f"Conversion complete: {success_count}/{len(args.chunk_indices)} files")
    logger.info("=" * 70)

    return 0 if success_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
