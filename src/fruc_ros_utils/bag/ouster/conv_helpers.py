"""Ouster LiDAR conversion helpers for ROS 2 → ROS 1 bag processing."""

import importlib
import json
import logging
import re
import shutil
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy.serialization
from sensor_msgs.msg import PointCloud2

try:
    import rosbag2_py
    HAS_ROSBAG2 = True
except Exception:
    rosbag2_py = None
    HAS_ROSBAG2 = False

from fruc_ros_utils.bag.ros2_inspector import open_ros2_reader
from fruc_ros_utils.bag.ros2_topic_filter import _source_topic_types
from fruc_ros_utils.utils.logging_utils import get_logger

logger = get_logger("Ros2utils", level="INFO", log_file=None)
warn = logger.warning


def _coerce_int(value) -> Optional[int]:
    try:
        if value is None or isinstance(value, bool):
            return None
        if isinstance(value, str):
            value = value.strip()
            if not value:
                return None
        return int(value)
    except Exception:
        return None


def _parse_ouster_lidar_mode(lidar_mode: Optional[str]) -> Dict[str, Optional[int]]:
    if not isinstance(lidar_mode, str):
        return {"columns_per_frame": None, "scan_rate_hz": None}
    match = re.search(r"(\d+)\s*x\s*(\d+)", lidar_mode)
    if not match:
        return {"columns_per_frame": None, "scan_rate_hz": None}
    return {
        "columns_per_frame": int(match.group(1)),
        "scan_rate_hz": int(match.group(2)),
    }


def _parse_ouster_channel_count(model: Optional[str]) -> Optional[int]:
    if not isinstance(model, str):
        return None
    match = re.search(r"OS-\d+-(\d+)", model)
    if not match:
        return None
    try:
        return int(match.group(1))
    except Exception:
        return None


def _get_msg_class(type_str: str):
    try:
        pkg, _, msg_name = type_str.partition("/msg/")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg_name)
    except Exception as e:
        warn(f"Cannot import {type_str}: {e}")
        return None


def _first_json_string_topic(bag_path: str, topic_name: str) -> Optional[dict]:
    try:
        topic_types = _source_topic_types(bag_path)
        type_str = topic_types.get(topic_name)
        if not type_str:
            return None
        msg_cls = _get_msg_class(type_str)
        if msg_cls is None:
            return None
        reader = open_ros2_reader(bag_path)
        while reader.has_next():
            cur_topic, data, _ = reader.read_next()
            if cur_topic != topic_name:
                continue
            msg = rclpy.serialization.deserialize_message(data, msg_cls)
            payload = getattr(msg, "data", None)
            if isinstance(payload, (bytes, bytearray)):
                payload = payload.decode("utf-8", errors="ignore")
            if isinstance(payload, str):
                return json.loads(payload)
            return None
    except Exception:
        return None
    return None


def _ouster_packets_per_scan(bag_path: str, prefix: str) -> Dict[str, object]:
    metadata_topic = f"{prefix}/metadata"
    metadata = _first_json_string_topic(bag_path, metadata_topic)
    columns_per_packet = 16
    columns_per_frame = None
    model = None
    lidar_mode = None
    scan_rate_hz = None
    channel_count = None
    if isinstance(metadata, dict):
        fmt = metadata.get("format", {}) if isinstance(metadata.get("format", {}), dict) else {}
        sensor_info = (
            metadata.get("sensor_info", {})
            if isinstance(metadata.get("sensor_info", {}), dict)
            else {}
        )
        config_params = (
            metadata.get("config_params", {})
            if isinstance(metadata.get("config_params", {}), dict)
            else {}
        )

        model = (
            sensor_info.get("prod_line")
            or sensor_info.get("product_line")
            or metadata.get("prod_line")
            or metadata.get("product_line")
        )
        lidar_mode = (
            config_params.get("lidar_mode")
            or sensor_info.get("lidar_mode")
            or metadata.get("lidar_mode")
        )

        columns_per_frame = (
            fmt.get("columns_per_frame")
            or config_params.get("columns_per_frame")
            or sensor_info.get("columns_per_frame")
            or metadata.get("columns_per_frame")
        )
        columns_per_packet = (
            fmt.get("columns_per_packet")
            or config_params.get("columns_per_packet")
            or sensor_info.get("columns_per_packet")
            or metadata.get("columns_per_packet")
            or columns_per_packet
        )

        mode_info = _parse_ouster_lidar_mode(lidar_mode)
        if columns_per_frame is None:
            columns_per_frame = mode_info.get("columns_per_frame")
        scan_rate_hz = mode_info.get("scan_rate_hz")
        channel_count = _parse_ouster_channel_count(model)

    cpf = _coerce_int(columns_per_frame)
    cpp = _coerce_int(columns_per_packet)
    if cpp is None:
        cpp = 16
    if cpf is None or cpp <= 0:
        packets_per_scan = None
    else:
        ratio = float(cpf) / float(cpp)
        rounded = int(round(ratio))
        if abs(ratio - float(rounded)) > 1e-6:
            packets_per_scan = None
        else:
            packets_per_scan = max(1, rounded)
    return {
        "metadata_topic": metadata_topic,
        "metadata_found": isinstance(metadata, dict),
        "model": model,
        "channel_count": channel_count,
        "lidar_mode": lidar_mode,
        "scan_rate_hz": scan_rate_hz,
        "columns_per_frame": cpf,
        "columns_per_packet": cpp,
        "packets_per_scan": packets_per_scan,
    }


def _pick_ouster_points_topic(dst_counts: Dict[str, int], prefix: str) -> Optional[str]:
    direct_candidates = [
        f"{prefix}/points/corrected",
        f"{prefix}/points",
    ]
    for topic in direct_candidates:
        if topic in dst_counts:
            return topic
    prefix_norm = f"{prefix}/"
    for topic in dst_counts:
        if topic.startswith(prefix_norm) and "/points" in topic:
            return topic
    return None


def _restore_lidar_fields_from_mcap(mcap_path: str, bag_path: str, topic: str) -> None:
    """Extract custom LiDAR fields (ring, reflectivity) from MCAP and inject into ROS1 bag."""
    if not HAS_ROSBAG2:
        warn("rosbag2_py not available; skipping LiDAR field restoration")
        return

    try:
        import rosbag
        from sensor_msgs.msg import PointField
    except ImportError:
        warn("rosbag not available; skipping LiDAR field restoration")
        return

    storage = rosbag2_py.StorageOptions(uri=mcap_path, storage_id="mcap")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    topics_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics_map:
        warn(f"Topic '{topic}' not found in MCAP")
        return

    ring_arrays = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        try:
            msg = rclpy.serialization.deserialize_message(data, PointCloud2)
            for field in msg.fields:
                if field.name == "ring":
                    num_points = len(msg.data) // msg.point_step
                    ring_data = np.frombuffer(
                        msg.data, dtype=np.uint16, count=num_points, offset=field.offset
                    )
                    ring_arrays.append(ring_data.copy())
                    break
        except Exception:
            continue

    if not ring_arrays:
        warn(f"No ring field found in MCAP topic '{topic}'")
        return

    logger.debug("Extracted %d ring arrays from MCAP topic '%s'", len(ring_arrays), topic)

    ring_idx = 0
    in_bag = rosbag.Bag(bag_path, "r")
    temp_bag_path = bag_path + ".tmp"
    out_bag = rosbag.Bag(temp_bag_path, "w")

    for in_topic, msg, t in in_bag.read_messages():
        if in_topic == topic and hasattr(msg, "fields") and ring_idx < len(ring_arrays):
            has_ring = any(f.name == "ring" for f in msg.fields)
            if not has_ring:
                ring_field = PointField(
                    name="ring",
                    offset=msg.point_step,
                    datatype=PointField.UINT16,
                    count=1,
                )
                msg.fields.append(ring_field)
                msg.point_step += 2
                ring_bytes = ring_arrays[ring_idx].tobytes()
                msg.data += ring_bytes
                msg.row_step = msg.width * msg.point_step
                ring_idx += 1
        out_bag.write(in_topic, msg, t)

    in_bag.close()
    out_bag.close()
    shutil.move(temp_bag_path, bag_path)
