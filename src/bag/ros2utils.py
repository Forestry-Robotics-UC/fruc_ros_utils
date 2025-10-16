#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT
#
# Description:
#   Multi-action utilities for ROS 2 bags (.db3).  Mirrors the ROS 1 bagutils
#   structure and uses the same logging system from utils.logging_utils.
#
#   Provides: topic listing, duration computation, generic JSON extraction.
#

import argparse
import sys
import json
import traceback
import sqlite3
import importlib
from pathlib import Path
from typing import Dict, Optional

# ===== ROS 2 =====
import rosbag2_py
import rclpy.serialization

# ===== Internal logging (shared with ROS 1 utils) =====
from utils.logging_utils import get_logger

# --------------------------------------------------------------------------- #
#                           LOGGER INITIALIZATION                             #
# --------------------------------------------------------------------------- #
_default_level = "INFO"
_default_file  = None
logger = get_logger("Ros2utils", level=_default_level, log_file=_default_file)

# convenience aliases (consistent with bagutils)
log, warn, err = (
    logger.info,
    logger.warning,
    logger.error,
)

# --------------------------------------------------------------------------- #
#                              CORE CLASS                                     #
# --------------------------------------------------------------------------- #
class Ros2BagUtils:
    """Utility class for processing ROS 2 bags (rosbag2 SQLite3)."""

    # ---------- internal helpers ----------
    def _open_reader(self, path: str):
        """Return an opened SequentialReader for a .db3 bag or folder."""
        bag_path = Path(path)
        if bag_path.is_dir():
            db3 = list(bag_path.glob("*.db3"))
            if not db3:
                raise FileNotFoundError(f"No .db3 file found in {bag_path}")
            bag_path = db3[0]
        storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
        converter = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, converter)
        return reader

    # ---------- actions ----------
    def list_topics(self, bag_path: str) -> Dict[str, str]:
        reader = self._open_reader(bag_path)
        topics = reader.get_all_topics_and_types()
        out = {t.name: t.type for t in topics}
        for name, ttype in out.items():
            logger.info("%-60s %s", name, ttype)
        return out

    def bag_duration(self, bag_path: str) -> float:
        """Compute duration directly from SQLite timestamps."""
        db3 = Path(bag_path)
        if db3.is_dir():
            db3 = next(db3.glob("*.db3"))
        conn = sqlite3.connect(db3)
        cur = conn.cursor()
        cur.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages;")
        start, end = cur.fetchone()
        conn.close()
        if not start or not end:
            warn("No messages found in bag")
            return 0.0
        dur = (end - start) / 1e9
        log(f"Duration: {dur:.3f} s")
        return dur

    # ---------- JSON extraction ----------
    def _get_msg_class(self, type_str: str):
        try:
            pkg, _, msg_name = type_str.partition("/msg/")
            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, msg_name)
        except Exception as e:
            warn(f"Cannot import {type_str}: {e}")
            return None

    def _try_extract_json(self, msg):
        for field in ("json_data", "data", "info", "payload"):
            if hasattr(msg, field):
                val = getattr(msg, field)
                if isinstance(val, str) and "{" in val:
                    try:
                        return json.loads(val)
                    except Exception:
                        continue
        return None

    def extract_json(self, bag_path: str, topic: Optional[str] = None, out_file: Optional[str] = None):
        reader = self._open_reader(bag_path)
        topics = reader.get_all_topics_and_types()
        topic_map = {t.name: t.type for t in topics}
        if topic and topic not in topic_map:
            err(f"Topic '{topic}' not found.")
            return

        results, count, json_count = [], 0, 0
        while reader.has_next():
            topic_name, data, ts = reader.read_next()
            if topic and topic_name != topic:
                continue
            cls = self._get_msg_class(topic_map[topic_name])
            if not cls:
                continue
            try:
                msg = rclpy.serialization.deserialize_message(data, cls)
                js = self._try_extract_json(msg)
                if js:
                    results.append({"timestamp": ts, "topic": topic_name, "data": js})
                    json_count += 1
            except Exception as e:
                warn(f"Failed to deserialize or parse on {topic_name}: {e}")
            count += 1

        log(f"Processed {count} msgs, extracted {json_count} JSON entries")
        if out_file:
            with open(out_file, "w") as f:
                json.dump(results, f, indent=2)
            log(f"Saved JSON output → {out_file}")
        else:
            if results:
                import pprint
                pprint.pprint(results[:3])
                if json_count > 3:
                    log(f"(showing 3 of {json_count})")
            else:
                warn("No JSON content extracted")

# --------------------------------------------------------------------------- #
#                               CLI PARSER                                    #
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS 2 bag utilities (Humble)")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    parser.add_argument("--log-file", default=None, help="Optional log file")

    sub = parser.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list_topics", help="List all topics in a bag")
    sp.add_argument("--bag", required=True)

    sp = sub.add_parser("duration", help="Compute duration of a bag")
    sp.add_argument("--bag", required=True)

    sp = sub.add_parser("extract_json", help="Extract JSON strings from messages")
    sp.add_argument("--bag", required=True)
    sp.add_argument("--topic", help="Specific topic (optional)")
    sp.add_argument("--out", help="Output JSON file")

    return parser

# --------------------------------------------------------------------------- #
#                               MAIN ENTRY                                    #
# --------------------------------------------------------------------------- #
def main():
    args = build_parser().parse_args()

    # reconfigure shared logger according to CLI options
    global logger, log, warn, err
    logger = get_logger("Ros2utils", level=args.log_level.upper(), log_file=args.log_file)
    log, warn, err = (logger.info, logger.warning, logger.error)

    utils = Ros2BagUtils()

    try:
        if args.cmd == "list_topics":
            utils.list_topics(args.bag)
        elif args.cmd == "duration":
            utils.bag_duration(args.bag)
        elif args.cmd == "extract_json":
            utils.extract_json(args.bag, args.topic, args.out)
        else:
            err(f"Unknown command: {args.cmd}")
    except KeyboardInterrupt:
        warn("Interrupted by user")
    except Exception as e:
        err(f"Fatal error: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
