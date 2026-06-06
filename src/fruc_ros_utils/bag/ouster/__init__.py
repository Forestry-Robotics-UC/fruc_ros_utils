"""Ouster LiDAR helpers: decode, encode, and conversion utilities."""

from .decode import OusterPacketDecoder
from .encode import OusterPointCloudEncoder
from .conv_helpers import (
    _coerce_int,
    _parse_ouster_lidar_mode,
    _parse_ouster_channel_count,
    _get_msg_class,
    _first_json_string_topic,
    _ouster_packets_per_scan,
    _pick_ouster_points_topic,
    _restore_lidar_fields_from_mcap,
)

__all__ = [
    "OusterPacketDecoder",
    "OusterPointCloudEncoder",
    "_coerce_int",
    "_parse_ouster_lidar_mode",
    "_parse_ouster_channel_count",
    "_get_msg_class",
    "_first_json_string_topic",
    "_ouster_packets_per_scan",
    "_pick_ouster_points_topic",
    "_restore_lidar_fields_from_mcap",
]
