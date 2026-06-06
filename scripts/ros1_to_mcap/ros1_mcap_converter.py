#!/usr/bin/env python3
"""Convert ROS1 bags directly to MCAP files using rosbags Python API."""

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

from _deps import ensure_rosbags as _ensure_rosbags


def convert_ros1_chunk_to_mcap(bag_path: Path, output_path: Path) -> bool:
    """Convert ROS1 bag to MCAP using Python API."""
    from rosbags.rosbag1 import Reader
    from mcap.writer import Writer as MCAPWriter
    from mcap.records import Message, Schema, Channel

    if not bag_path.exists():
        logger.error(f"Input not found: {bag_path}")
        return False

    try:
        logger.info(f"Converting: {bag_path.name} → {output_path.name}")

        # Read ROS1 bag
        with Reader(str(bag_path)) as reader:
            connections = {conn.id: conn for conn in reader.connections}
            logger.info(f"  Topics: {len(connections)}")

            # Write MCAP file
            with open(output_path, "wb") as f:
                writer = MCAPWriter(f)
                writer.start(profile="ros2")

                # Register channels (topics/schemas)
                channel_ids = {}
                for conn_id, conn in connections.items():
                    schema = Schema(
                        name=conn.msgtype,
                        encoding="ros2msg",
                        data=conn.msgdef.encode("utf-8"),
                    )
                    schema_id = writer.write_schema(schema)

                    channel = Channel(
                        topic=conn.topic,
                        schema_id=schema_id,
                        message_encoding="cdr",
                    )
                    channel_id = writer.write_channel(channel)
                    channel_ids[conn_id] = channel_id

                # Write messages
                msg_count = 0
                for conn_id, timestamp_ns, rawdata in reader.messages():
                    channel_id = channel_ids[conn_id]

                    message = Message(
                        channel_id=channel_id,
                        publish_time=timestamp_ns,
                        data=rawdata,
                    )
                    writer.write_message(message)
                    msg_count += 1

                    if msg_count % 5000 == 0:
                        logger.debug(f"  Messages: {msg_count}")

                writer.finish()

            logger.info(f"  ✓ Wrote {msg_count} messages")
            return True

    except Exception as e:
        logger.error(f"Conversion failed: {e}", exc_info=True)
        if output_path.exists():
            output_path.unlink()
        return False


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert ROS1 bags to MCAP files"
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
    logger.info("ROS1 Chunks → MCAP Conversion (Direct API)")
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

        if convert_ros1_chunk_to_mcap(bag_file, mcap_file):
            success += 1
        logger.info("")

    logger.info("=" * 70)
    logger.info(f"Complete: {success}/{len(args.chunk_indices)} chunks converted")
    logger.info("=" * 70)

    return 0 if success > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
