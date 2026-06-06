"""ROS bag utility modules for ROS1/ROS2 processing and conversion."""

from .ouster import OusterPointCloudEncoder
from .bagutils import RosbagUtils
from .ros2utils import Ros2BagUtils

# ROS 2 bag inspection
from .ros2_inspector import (
    open_ros2_reader,
    list_topics,
    bag_size_bytes,
    bag_size_bytes_raw,
    bag_duration,
    ensure_output_available,
    info,
)

# ROS 2 → ROS 1 conversion
from .ros2_converter import (
    convert_single_to_ros1,
    convert_single_to_ros1_with_ouster_decode,
    convert_to_ros1_with_split,
    convert_existing_split_members_to_ros1,
)

__all__ = [
    # Classes
    "OusterPointCloudEncoder",
    "RosbagUtils",
    "Ros2BagUtils",
    # Inspection
    "open_ros2_reader",
    "list_topics",
    "bag_size_bytes",
    "bag_size_bytes_raw",
    "bag_duration",
    "ensure_output_available",
    "info",
    # Conversion
    "convert_single_to_ros1",
    "convert_single_to_ros1_with_ouster_decode",
    "convert_to_ros1_with_split",
    "convert_existing_split_members_to_ros1",
]
