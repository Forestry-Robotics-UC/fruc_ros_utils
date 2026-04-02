#!/usr/bin/env python3
"""Repack PointCloud2 messages in a ROS1 bag for iKalibr."""

from __future__ import annotations

import argparse
from pathlib import Path

try:
    import argcomplete
except Exception:
    argcomplete = None


def build_parser(enable_shell_completion: bool = True) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Repack PointCloud2 fields for iKalibr")
    parser.add_argument("--in", dest="in_bag", required=True, help="Input ROS1 .bag")
    parser.add_argument("--out", dest="out_bag", required=True, help="Output ROS1 .bag")
    parser.add_argument(
        "--topic",
        dest="topic",
        default="/ouster/points/corrected",
        help="PointCloud2 topic to repack",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output if exists")
    parser.add_argument(
        "--ring-dtype",
        dest="ring_dtype",
        choices=("auto", "uint8", "uint16"),
        default="auto",
        help="Force ring field datatype (default=auto detect from input)",
    )
    if enable_shell_completion and argcomplete:
        argcomplete.autocomplete(parser)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        import rosbag
        from sensor_msgs.msg import PointField
        import sensor_msgs.point_cloud2 as pc2
    except Exception as exc:
        print("ERROR: This script requires ROS1 python packages (rosbag, sensor_msgs):", exc)
        return 2

    in_path = Path(args.in_bag)
    out_path = Path(args.out_bag)
    if not in_path.exists():
        print(f"Input bag not found: {in_path}")
        return 2
    if out_path.exists() and not args.overwrite:
        print(f"Output bag already exists: {out_path} (use --overwrite to replace)")
        return 2

    topic = args.topic
    target_fields = ["x", "y", "z", "intensity", "ring", "t", "range"]

    dt_float32 = PointField.FLOAT32
    dt_uint8 = PointField.UINT8
    dt_uint16 = PointField.UINT16
    dt_uint32 = PointField.UINT32

    ring_dtype = args.ring_dtype
    if ring_dtype == "auto":
        try:
            with rosbag.Bag(str(in_path), "r") as bag:
                for _topic, message, _stamp in bag.read_messages(topics=[topic]):
                    for field in message.fields:
                        if field.name != "ring":
                            continue
                        if field.datatype in (PointField.UINT8, PointField.INT8, 2, 1):
                            ring_dtype = "uint8"
                        else:
                            ring_dtype = "uint16"
                        break
                    break
        except Exception:
            ring_dtype = "uint16"

    ring_field_type = dt_uint16 if ring_dtype == "uint16" else dt_uint8
    target_pointfields = [
        PointField(name="x", offset=0, datatype=dt_float32, count=1),
        PointField(name="y", offset=4, datatype=dt_float32, count=1),
        PointField(name="z", offset=8, datatype=dt_float32, count=1),
        PointField(name="intensity", offset=12, datatype=dt_float32, count=1),
        PointField(name="ring", offset=16, datatype=ring_field_type, count=1),
        PointField(name="t", offset=18, datatype=dt_uint32, count=1),
        PointField(name="range", offset=22, datatype=dt_float32, count=1),
    ]

    in_bag = rosbag.Bag(str(in_path), "r")
    out_bag = rosbag.Bag(str(out_path), "w")

    total_messages = 0
    repacked_messages = 0
    try:
        for current_topic, message, stamp in in_bag.read_messages():
            total_messages += 1
            if current_topic != topic:
                out_bag.write(current_topic, message, stamp)
                continue

            source_names = [field.name for field in message.fields]
            name_to_idx = {name: idx for idx, name in enumerate(source_names)}
            points = []
            for point in pc2.read_points(message, skip_nans=False):
                output_point = []
                for field_name in target_fields:
                    if field_name in name_to_idx:
                        value = point[name_to_idx[field_name]]
                    elif field_name in ("x", "y", "z"):
                        value = float("nan")
                    else:
                        value = 0

                    if field_name == "ring":
                        try:
                            value = int(value or 0)
                            value = value & (0xFFFF if ring_field_type == dt_uint16 else 0xFF)
                        except Exception:
                            value = 0
                    output_point.append(value)
                points.append(tuple(output_point))

            new_message = pc2.create_cloud(message.header, target_pointfields, points)
            out_bag.write(topic, new_message, stamp)
            repacked_messages += 1
    finally:
        in_bag.close()
        out_bag.close()

    print(f"Processed {total_messages} messages, repacked {repacked_messages} on '{topic}'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
