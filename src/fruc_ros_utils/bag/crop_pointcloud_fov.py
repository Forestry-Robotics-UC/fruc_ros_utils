#!/usr/bin/env python3
"""Crop a ROS1 PointCloud2 topic to a horizontal field of view."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Crop a ROS1 PointCloud2 bag topic by horizontal FOV"
    )
    parser.add_argument("--in", dest="in_bag", required=True, help="Input ROS1 .bag")
    parser.add_argument("--out", dest="out_bag", required=True, help="Output ROS1 .bag")
    parser.add_argument("--topic", default="/ouster/points", help="PointCloud2 topic to crop")
    parser.add_argument(
        "--fov-deg",
        type=float,
        default=120.0,
        help="Horizontal field of view to keep",
    )
    parser.add_argument(
        "--center-deg",
        type=float,
        default=0.0,
        help="Center azimuth in degrees",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output if it exists")
    return parser


def _wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        import rosbag
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs.point_cloud2 as pc2
    except Exception as exc:
        print("ERROR: This script requires ROS1 python packages:", exc)
        return 2

    in_path = Path(args.in_bag)
    out_path = Path(args.out_bag)
    if not in_path.exists():
        print(f"Input bag not found: {in_path}")
        return 2
    if out_path.exists() and not args.overwrite:
        print(f"Output bag already exists: {out_path} (use --overwrite to replace)")
        return 2
    if args.fov_deg <= 0.0 or args.fov_deg > 360.0:
        print(f"Invalid --fov-deg {args.fov_deg}; expected 0 < fov <= 360")
        return 2

    keep_half_span = float(args.fov_deg) / 2.0
    center_deg = float(args.center_deg)
    target_topic = args.topic

    total_messages = 0
    cropped_messages = 0
    total_points = 0
    kept_points = 0

    with rosbag.Bag(str(in_path), "r") as in_bag, rosbag.Bag(str(out_path), "w") as out_bag:
        for topic, message, stamp in in_bag.read_messages():
            total_messages += 1
            if topic != target_topic:
                out_bag.write(topic, message, stamp)
                continue

            if not isinstance(message, PointCloud2):
                out_bag.write(topic, message, stamp)
                continue

            field_names = [field.name for field in message.fields]
            try:
                x_idx = field_names.index("x")
                y_idx = field_names.index("y")
            except ValueError as exc:
                raise RuntimeError(
                    f"PointCloud2 topic '{target_topic}' is missing x/y fields; cannot crop by azimuth."
                ) from exc

            selected_points = []
            for point in pc2.read_points(message, field_names=field_names, skip_nans=False):
                total_points += 1
                try:
                    x = float(point[x_idx])
                    y = float(point[y_idx])
                except Exception:
                    continue
                if not math.isfinite(x) or not math.isfinite(y):
                    continue

                azimuth_deg = math.degrees(math.atan2(y, x))
                delta_deg = _wrap_angle_deg(azimuth_deg - center_deg)
                if abs(delta_deg) <= keep_half_span:
                    selected_points.append(tuple(point))
                    kept_points += 1

            new_message = pc2.create_cloud(message.header, message.fields, selected_points)
            new_message.is_bigendian = message.is_bigendian
            new_message.is_dense = False
            out_bag.write(topic, new_message, stamp)
            cropped_messages += 1

    ratio = (100.0 * kept_points / total_points) if total_points else 0.0
    print(
        f"Processed {total_messages} messages, cropped {cropped_messages} PointCloud2 messages on '{target_topic}'."
    )
    print(
        f"Kept {kept_points}/{total_points} points within {args.fov_deg:.1f} deg centered at {center_deg:.1f} deg "
        f"({ratio:.2f}%)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
