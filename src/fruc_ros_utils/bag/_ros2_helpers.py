"""Stateless module-level helpers shared across ros2utils submodules."""

import re
from pathlib import Path
from typing import Dict, List, Optional, Union

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_ROS2_MSG_TYPE_SEPARATOR = "/msg/"

_SIZE_SUFFIXES: Dict[str, int] = {
    "": 1, "b": 1,
    "kb": 1000, "k": 1000,
    "mb": 1000 ** 2, "m": 1000 ** 2,
    "gb": 1000 ** 3, "g": 1000 ** 3,
    "tb": 1000 ** 4, "t": 1000 ** 4,
}

_DURATION_SUFFIXES: Dict[str, int] = {
    "": 1, "s": 1, "sec": 1, "secs": 1, "second": 1, "seconds": 1,
    "m": 60, "min": 60, "mins": 60, "minute": 60, "minutes": 60,
    "h": 3600, "hr": 3600, "hrs": 3600, "hour": 3600, "hours": 3600,
}

_FAILED_PARSE_MSGTYPE_RE = re.compile(r"Failed to parse\s+([A-Za-z0-9_]+/msg/[A-Za-z0-9_]+)")

_OPTIONAL_DROPPED_ROS2_TYPES = {
    "realsense2_camera_msgs/msg/Metadata",
    "realsense2_camera_msgs/msg/Extrinsics",
    "tf2_msgs/msg/TFMessage",
}

_FFMPEG_PACKET_ROS2_TYPES = {
    "ffmpeg_image_transport_msgs/msg/FFMPEGPacket",
    "ffmpeg_image_transport_msgs/FFMPEGPacket",
}

_STANDARD_ROS_TYPE_PREFIXES = (
    "sensor_msgs/",
    "std_msgs/",
    "geometry_msgs/",
    "nav_msgs/",
    "builtin_interfaces/",
    "rosgraph_msgs/",
    "tf2_msgs/",
    "diagnostic_msgs/",
    "visualization_msgs/",
    "shape_msgs/",
    "actionlib_msgs/",
    "stereo_msgs/",
    "trajectory_msgs/",
)

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def _make_progress_bar(
    desc: str,
    unit: str,
    total: Optional[int] = None,
    position: int = 0,
    leave: bool = True,
):
    try:
        from tqdm import tqdm
        display_unit = unit if not unit or unit.startswith(" ") else f" {unit}"
        return tqdm(
            desc=desc,
            unit=display_unit,
            total=total,
            dynamic_ncols=True,
            position=position,
            leave=leave,
        )
    except Exception:
        return None


def _debug_path_state(pathlike: Union[str, Path]) -> str:
    path = Path(pathlike)
    parts = [f"path={path}"]
    try:
        parts.append(f"resolved={path.resolve()}")
    except Exception:
        parts.append("resolved=<unavailable>")

    exists = path.exists()
    parts.append(f"exists={exists}")
    if not exists:
        return " ".join(parts)

    if path.is_file():
        parts.append("kind=file")
        parts.append(f"suffix={path.suffix or '<none>'}")
        try:
            parts.append(f"size={path.stat().st_size}")
        except Exception:
            parts.append("size=<unavailable>")
        return " ".join(parts)

    if path.is_dir():
        parts.append("kind=dir")
        metadata_path = path / "metadata.yaml"
        parts.append(f"metadata_yaml={metadata_path.exists()}")
        try:
            file_names = sorted(child.name for child in path.iterdir() if child.is_file())
            if file_names:
                preview = ", ".join(file_names[:5])
                if len(file_names) > 5:
                    preview += f", ... (+{len(file_names) - 5} more)"
                parts.append(f"files=[{preview}]")
        except Exception:
            parts.append("files=<unavailable>")
        return " ".join(parts)

    parts.append("kind=other")
    return " ".join(parts)


def _debug_topic_count_summary(counts: Dict[str, int], limit: int = 10) -> str:
    if not counts:
        return "0 topics"
    items = sorted(counts.items(), key=lambda item: (-int(item[1]), item[0]))
    preview = ", ".join(f"{topic}={count}" for topic, count in items[:limit])
    if len(items) > limit:
        preview += f", ... (+{len(items) - limit} more)"
    return f"{len(items)} topics [{preview}]"


def _is_standard_ros_msg_type(msg_type: Optional[str]) -> bool:
    if not isinstance(msg_type, str):
        return False
    return any(msg_type.startswith(prefix) for prefix in _STANDARD_ROS_TYPE_PREFIXES)


def _is_ffmpeg_packet_msg_type(msg_type: Optional[str]) -> bool:
    if not isinstance(msg_type, str):
        return False
    return msg_type.strip().replace(_ROS2_MSG_TYPE_SEPARATOR, "/") in _FFMPEG_PACKET_ROS2_TYPES


def _safe_rosbag2_metadata_scalar(value, default: str = "") -> str:
    if value is None:
        return default
    if isinstance(value, str):
        return value
    if isinstance(value, (int, float, bool)):
        return str(value)
    if isinstance(value, (list, tuple)):
        if not value:
            return default
        if all(isinstance(item, str) for item in value):
            return "\n".join(item for item in value if item)
        return default
    return default


def _merge_topic_lists(*topic_lists: Optional[List[str]]) -> Optional[List[str]]:
    merged: List[str] = []
    seen = set()
    for topic_list in topic_lists:
        for topic in topic_list or []:
            if topic in seen:
                continue
            merged.append(topic)
            seen.add(topic)
    return merged or None


def _parse_threshold_value(
    value,
    suffixes: Dict[str, int],
    label: str,
) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"Invalid {label}: {value}")
    if isinstance(value, (int, float)):
        parsed = int(float(value))
        return parsed if parsed > 0 else None

    raw = str(value).strip().lower()
    if not raw or raw == "0":
        return None

    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)\s*([a-z]+)?", raw)
    if not match:
        raise ValueError(f"Invalid {label}: {value}")

    suffix = (match.group(2) or "").lower()
    if suffix not in suffixes:
        raise ValueError(
            f"Invalid {label}: {value}. Supported suffixes: "
            f"{', '.join(sorted(k or '<none>' for k in suffixes))}"
        )

    parsed = int(float(match.group(1)) * suffixes[suffix])
    return parsed if parsed > 0 else None


def _parse_size_bytes(value) -> Optional[int]:
    return _parse_threshold_value(value, _SIZE_SUFFIXES, "size")


def _parse_duration_seconds(value) -> Optional[int]:
    return _parse_threshold_value(value, _DURATION_SUFFIXES, "duration")


def _format_bytes(value: int) -> str:
    size = float(max(0, int(value)))
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if size < 1000.0 or unit == "TB":
            return f"{size:.1f} {unit}" if unit != "B" else f"{int(size)} B"
        size /= 1000.0
    return f"{int(value)} B"


def _natural_sort_key(pathlike) -> List[Union[int, str]]:
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", str(pathlike))
        if part
    ]
