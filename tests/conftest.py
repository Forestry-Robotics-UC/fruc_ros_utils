"""
conftest.py — inject ROS1/ROS2 stubs before any fruc_ros_utils module is loaded,
and ensure src/ is on sys.path.
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

# Make sure the src layout is importable without installation.
_SRC = str(Path(__file__).resolve().parents[1] / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


# ---------------------------------------------------------------------------
# Stub registry
# ---------------------------------------------------------------------------
class _AnyMod(types.ModuleType):
    """A permissive stand-in for any ROS / C-extension module."""

    def __getattr__(self, name: str):
        key = f"{self.__name__}.{name}"
        if key not in sys.modules:
            child = _AnyMod(key)
            sys.modules[key] = child
        return sys.modules[key]

    def __call__(self, *a, **kw):
        return self

    def __iter__(self):
        return iter([])

    def __bool__(self):
        return False

    # Make subscription / annotation syntax work (e.g. List[str])
    def __getitem__(self, item):
        return self

    def __class_getitem__(cls, item):
        return cls

    # Context manager support (for rosbag.Bag used as 'with rosbag.open(...) as bag:')
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


_ROS1_STUBS = [
    "ouster", "ouster.sdk", "ouster.sdk.client", "ouster.sdk.core",
    "ouster.sdk._bindings", "ouster.sdk._bindings.client",
    "ouster.sdk.util",
    "ouster_sensor_msgs", "ouster_sensor_msgs.msg",
    "rosbag",
    "rospy",
    "genpy",
    "std_msgs", "std_msgs.msg",
    "sensor_msgs", "sensor_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "diagnostic_msgs", "diagnostic_msgs.msg",
    "tf2_ros",
    "tf2_sensor_msgs", "tf2_sensor_msgs.tf2_sensor_msgs",
    "cv_bridge",
    "image_geometry",
    "pyproj",
    "skimage", "skimage.measure", "skimage.filters", "skimage.morphology",
    "usb", "usb.core", "usb.util",
    # ROS2 stubs so ros2utils can be imported without crashing
    "rclpy", "rclpy.serialization",
    "rosbag2_py",
]


def _inject_stubs() -> None:
    for mod_name in _ROS1_STUBS:
        if mod_name not in sys.modules:
            try:
                __import__(mod_name)
            except ImportError:
                stub = _AnyMod(mod_name)
                sys.modules[mod_name] = stub
                # Also populate dotted parent chain
                parts = mod_name.split(".")
                for i in range(1, len(parts)):
                    parent_name = ".".join(parts[:i])
                    child_name = parts[i]
                    parent = sys.modules.get(parent_name)
                    if parent is not None and not isinstance(parent, types.ModuleType):
                        pass
                    elif parent is not None:
                        try:
                            setattr(parent, child_name, sys.modules[mod_name])
                        except (AttributeError, TypeError):
                            pass


# Inject immediately at collection time so that fruc_ros_utils imports work.
_inject_stubs()
