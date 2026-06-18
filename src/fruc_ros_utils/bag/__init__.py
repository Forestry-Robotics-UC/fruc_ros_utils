"""ROS bag utility modules for ROS1/ROS2 processing and conversion."""

# RosbagUtils is pure ROS1 — always safe to import eagerly.
from .bagutils import RosbagUtils

# Everything else in this package requires ROS2 (rclpy, ros2 message types, etc.).
# They are loaded on first attribute access so that importing this package in a
# ROS1-only environment (e.g. the Noetic demo container) does not fail.

_ROS2_SYMBOLS = {
    # (attribute_name, dotted_submodule, symbol_in_submodule)
    "OusterPointCloudEncoder": (".ouster", "OusterPointCloudEncoder"),
    "Ros2BagUtils":            (".ros2utils", "Ros2BagUtils"),
    "open_ros2_reader":        (".ros2_inspector", "open_ros2_reader"),
    "list_topics":             (".ros2_inspector", "list_topics"),
    "bag_size_bytes":          (".ros2_inspector", "bag_size_bytes"),
    "bag_size_bytes_raw":      (".ros2_inspector", "bag_size_bytes_raw"),
    "bag_duration":            (".ros2_inspector", "bag_duration"),
    "ensure_output_available": (".ros2_inspector", "ensure_output_available"),
    "info":                    (".ros2_inspector", "info"),
    "convert_single_to_ros1":                     (".ros2_converter", "convert_single_to_ros1"),
    "convert_single_to_ros1_with_ouster_decode":  (".ros2_converter", "convert_single_to_ros1_with_ouster_decode"),
    "convert_to_ros1_with_split":                 (".ros2_converter", "convert_to_ros1_with_split"),
    "convert_existing_split_members_to_ros1":     (".ros2_converter", "convert_existing_split_members_to_ros1"),
}


def __getattr__(name: str):
    if name not in _ROS2_SYMBOLS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    submodule_path, symbol = _ROS2_SYMBOLS[name]
    try:
        import importlib
        mod = importlib.import_module(submodule_path, package=__name__)
    except ModuleNotFoundError as exc:
        if "rclpy" in str(exc):
            raise ModuleNotFoundError(
                f"{name!r} requires the ROS2 Python package 'rclpy'. "
                "This feature is not available in the ROS1-only (Noetic) container."
            ) from exc
        raise
    obj = getattr(mod, symbol)
    # Cache so subsequent accesses don't re-import.
    globals()[name] = obj
    return obj


__all__ = [
    "RosbagUtils",
    # ROS2 symbols — available when rclpy is present
    "OusterPointCloudEncoder",
    "Ros2BagUtils",
    "open_ros2_reader",
    "list_topics",
    "bag_size_bytes",
    "bag_size_bytes_raw",
    "bag_duration",
    "ensure_output_available",
    "info",
    "convert_single_to_ros1",
    "convert_single_to_ros1_with_ouster_decode",
    "convert_to_ros1_with_split",
    "convert_existing_split_members_to_ros1",
]
