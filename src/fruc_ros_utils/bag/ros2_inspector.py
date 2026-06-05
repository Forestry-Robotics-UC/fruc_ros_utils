"""ROS 2 bag inspection, I/O, and typestore helpers."""

import logging
import shutil
import sqlite3
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Union

import yaml

try:
    import rosbag2_py
    HAS_ROSBAG2 = True
except Exception:
    rosbag2_py = None
    HAS_ROSBAG2 = False

from fruc_ros_utils.bag._ros2_helpers import (
    _debug_path_state,
    _format_bytes,
    _is_standard_ros_msg_type,
    _safe_rosbag2_metadata_scalar,
)
from fruc_ros_utils.utils.logging_utils import get_logger

logger = get_logger("Ros2utils", level="INFO", log_file=None)
log = logger.info
warn = logger.warning


# ---------------------------------------------------------------------------
# Debug log helpers
# ---------------------------------------------------------------------------

def _debug_log_topic_descriptions(context: str, topics: List[object], limit: int = 20) -> None:
    if not logger.isEnabledFor(logging.DEBUG):
        return
    logger.debug("%s: topic_count=%d", context, len(topics))
    for topic in topics[:limit]:
        logger.debug(
            "%s: topic=%s type=%s serialization=%s",
            context,
            getattr(topic, "name", "<unknown>"),
            getattr(topic, "type", "<unknown>"),
            getattr(topic, "serialization_format", "<unknown>"),
        )
    if len(topics) > limit:
        logger.debug("%s: topic list truncated after %d entries", context, limit)


def _debug_log_opened_ros2_reader(reader, path: Union[str, Path], context: str) -> None:
    if not logger.isEnabledFor(logging.DEBUG):
        return
    logger.debug("%s: %s", context, _debug_path_state(path))
    try:
        topics = list(reader.get_all_topics_and_types())
    except Exception:
        logger.debug("%s: failed to enumerate topics", context, exc_info=True)
        topics = []
    else:
        _debug_log_topic_descriptions(context, topics)

    get_metadata = getattr(reader, "get_metadata", None)
    if not callable(get_metadata):
        return
    try:
        metadata = get_metadata()
        logger.debug(
            "%s: metadata storage=%r duration=%r message_count=%r relative_files=%r",
            context,
            getattr(metadata, "storage_identifier", None),
            getattr(getattr(metadata, "duration", None), "nanoseconds", None),
            getattr(metadata, "message_count", None),
            getattr(metadata, "relative_file_paths", None),
        )
    except Exception:
        logger.debug("%s: failed to read rosbag2 metadata", context, exc_info=True)


def _debug_log_ros2_bag_summary(bag_path: Union[str, Path], context: str) -> None:
    if not logger.isEnabledFor(logging.DEBUG):
        return
    try:
        reader = open_ros2_reader(str(bag_path))
    except Exception:
        logger.debug(
            "%s: unable to open ROS2 bag for debug summary: %s",
            context,
            _debug_path_state(bag_path),
            exc_info=True,
        )
        return
    try:
        _debug_log_opened_ros2_reader(reader, bag_path, context)
    finally:
        close_reader = getattr(reader, "close", None)
        if callable(close_reader):
            close_reader()


def _debug_log_subprocess_result(
    context: str, cmd: List[str], result: subprocess.CompletedProcess
) -> None:
    if not logger.isEnabledFor(logging.DEBUG):
        return
    logger.debug("%s: returncode=%s cmd=%s", context, result.returncode, cmd)
    stdout = (result.stdout or "").strip()
    stderr = (result.stderr or "").strip()
    if stdout:
        logger.debug("%s stdout:\n%s", context, stdout)
    if stderr:
        logger.debug("%s stderr:\n%s", context, stderr)


# ---------------------------------------------------------------------------
# Typestore utilities
# ---------------------------------------------------------------------------

def _resolve_rosbags_typestore(typestore: object):
    if not isinstance(typestore, str):
        return typestore
    try:
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            f"rosbags.typesys is required to resolve typestore '{typestore}': {exc}"
        ) from exc

    normalized = typestore.strip().replace("-", "_").replace(" ", "_")
    members = getattr(Stores, "__members__", {})

    alias_candidates = {
        "ros2_jazzy": ["ros2_jazzy", "ros2_iron", "ros2_humble", "ros2_galactic", "ros2_foxy"],
        "ros2_iron": ["ros2_iron", "ros2_humble", "ros2_galactic", "ros2_foxy"],
        "ros2_humble": ["ros2_humble", "ros2_galactic", "ros2_foxy"],
        "ros1_noetic": ["ros1_noetic"],
    }

    for candidate in (normalized, normalized.upper(), normalized.lower()):
        if candidate in members:
            return get_typestore(members[candidate])
        attr = getattr(Stores, candidate, None)
        if attr is not None:
            return get_typestore(attr)

    for alias in alias_candidates.get(normalized.lower(), []):
        for candidate in (alias, alias.upper(), alias.lower()):
            if candidate in members:
                logger.warning(
                    "rosbags typestore '%s' is unavailable; falling back to '%s'.",
                    typestore,
                    candidate,
                )
                return get_typestore(members[candidate])
            attr = getattr(Stores, candidate, None)
            if attr is not None:
                logger.warning(
                    "rosbags typestore '%s' is unavailable; falling back to '%s'.",
                    typestore,
                    candidate,
                )
                return get_typestore(attr)

    for name, store in members.items():
        if name.lower() == normalized.lower():
            return get_typestore(store)

    raise ValueError(f"Unknown rosbags typestore: {typestore}")


def _ensure_ros1_tf2_typestore(dst_store) -> None:
    if dst_store is None:
        return
    store_types = getattr(dst_store, "types", None)
    if isinstance(store_types, dict) and "tf2_msgs/msg/TFMessage" in store_types:
        return
    try:
        from rosbags.typesys import get_types_from_msg
        tf_msgdef = "geometry_msgs/TransformStamped[] transforms\n"
        dst_store.register(get_types_from_msg(tf_msgdef, "tf2_msgs/msg/TFMessage"))
        logger.debug("Registered missing ROS1 typestore definition for tf2_msgs/msg/TFMessage")
    except Exception:
        logger.debug(
            "Failed to register tf2_msgs/msg/TFMessage in ROS1 typestore",
            exc_info=True,
        )


def _cleanup_partial_output(dst_path: Union[str, Path], context: str = "") -> None:
    dst = Path(dst_path)
    if not dst.exists():
        return
    try:
        if dst.is_dir():
            shutil.rmtree(dst)
        else:
            dst.unlink()
        logger.debug(
            "Removed partial conversion output %s%s",
            dst,
            f" after {context}" if context else "",
        )
    except Exception:
        logger.debug(
            "Failed to remove partial conversion output %s%s",
            dst,
            f" after {context}" if context else "",
            exc_info=True,
        )


# ---------------------------------------------------------------------------
# Reader factory
# ---------------------------------------------------------------------------

def open_ros2_reader(path: str):
    """Return an opened SequentialReader for a .mcap, .db3 bag or folder."""
    if rosbag2_py is None:
        raise RuntimeError(
            "rosbag2_py is required in this environment to read ROS 2 bags. "
            "Use the jazzy container/image."
        )
    bag_path = Path(path)
    logger.debug("Opening ROS2 reader for %s", _debug_path_state(bag_path))
    if bag_path.is_dir():
        storage_id = ""
        uri = str(bag_path)
    else:
        ext = bag_path.suffix.lower()
        storage_id = {".mcap": "mcap", ".db3": "sqlite3"}.get(ext, "")
        uri = str(bag_path)

    storage_ids = [storage_id]
    if storage_id == "mcap":
        storage_ids.append("")

    last_error: Optional[Exception] = None
    for candidate_storage_id in storage_ids:
        storage = rosbag2_py.StorageOptions(uri=uri, storage_id=candidate_storage_id)
        converter = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        logger.debug(
            "Trying rosbag2_py.SequentialReader.open uri=%s storage_id=%r",
            uri,
            candidate_storage_id,
        )
        try:
            reader.open(storage, converter)
            _debug_log_opened_ros2_reader(
                reader,
                bag_path,
                context=f"Opened ROS2 reader with storage_id={candidate_storage_id!r}",
            )
            return reader
        except Exception as e:  # noqa: BLE001
            last_error = e
            logger.debug(
                "SequentialReader open failed for %s with storage_id=%r: %s",
                path,
                candidate_storage_id,
                e,
                exc_info=True,
            )
            continue

    raise RuntimeError(f"Failed to open ROS2 reader for {bag_path}: {last_error}")


# ---------------------------------------------------------------------------
# Bag inspection
# ---------------------------------------------------------------------------

def list_topics(bag_path: str) -> Dict[str, str]:
    reader = open_ros2_reader(bag_path)
    topics = reader.get_all_topics_and_types()
    out = {t.name: t.type for t in topics}
    for name, ttype in out.items():
        logger.info("%-60s %s", name, ttype)
    return out


def bag_size_bytes_raw(bag_path: Union[str, Path]) -> int:
    bag = Path(bag_path)
    if bag.is_file():
        return int(bag.stat().st_size)
    if bag.is_dir():
        return int(sum(path.stat().st_size for path in bag.rglob("*") if path.is_file()))
    raise FileNotFoundError(f"Bag not found: {bag}")


def bag_size_bytes(bag_path: str) -> int:
    total = bag_size_bytes_raw(bag_path)
    log(f"Size: {total} bytes ({_format_bytes(total)})")
    return int(total)


def bag_duration(bag_path: str) -> float:
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


def ensure_output_available(dst_path: str, overwrite: bool) -> None:
    dst = Path(dst_path)
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.exists():
        return
    if not overwrite:
        raise FileExistsError(
            f"Output path '{dst}' exists. Use --overwrite to replace it."
        )
    try:
        if dst.is_dir():
            shutil.rmtree(dst)
        else:
            dst.unlink()
        logger.debug("Removed existing output path for overwrite: %s", dst)
    except Exception as e:
        raise RuntimeError(f"Unable to clear existing output path '{dst}': {e}")


def _storage_id_for_bag(bag_path: str) -> str:
    bag = Path(bag_path)
    if bag.is_file():
        storage_id = {".mcap": "mcap", ".db3": "sqlite3"}.get(bag.suffix.lower(), "")
        if storage_id:
            return storage_id

    reader = open_ros2_reader(bag_path)
    metadata = reader.get_metadata()
    storage_id = getattr(metadata, "storage_identifier", "") or ""
    if storage_id:
        return storage_id

    raise RuntimeError(f"Could not determine storage backend for bag: {bag_path}")


def _single_file_rosbag2_metadata(bag_path: str) -> Dict[str, object]:
    reader = open_ros2_reader(bag_path)
    try:
        topics = list(reader.get_all_topics_and_types())
        topic_entries = []
        for topic in topics:
            topic_entries.append(
                {
                    "topic_metadata": {
                        "name": topic.name,
                        "type": topic.type,
                        "serialization_format": _safe_rosbag2_metadata_scalar(
                            getattr(topic, "serialization_format", ""),
                            default="cdr",
                        ) or "cdr",
                        "offered_qos_profiles": _safe_rosbag2_metadata_scalar(
                            getattr(topic, "offered_qos_profiles", ""),
                            default="",
                        ),
                        "type_description_hash": _safe_rosbag2_metadata_scalar(
                            getattr(topic, "type_description_hash", ""),
                            default="",
                        ),
                    },
                    "message_count": 0,
                }
            )

        first_ts = None
        last_ts = None
        message_count = 0
        topic_counts: Dict[str, int] = defaultdict(int)
        while reader.has_next():
            topic_name, _data, ts = reader.read_next()
            topic_counts[topic_name] += 1
            message_count += 1
            if first_ts is None:
                first_ts = ts
            last_ts = ts

        for entry in topic_entries:
            entry["message_count"] = topic_counts.get(entry["topic_metadata"]["name"], 0)

        first_ts = int(first_ts or 0)
        last_ts = int(last_ts or first_ts)
        duration_ns = max(0, last_ts - first_ts)
        bag_name = Path(bag_path).name
        metadata = {
            "rosbag2_bagfile_information": {
                "version": 8,
                "storage_identifier": _storage_id_for_bag(bag_path),
                "duration": {"nanoseconds": duration_ns},
                "starting_time": {"nanoseconds_since_epoch": first_ts},
                "message_count": message_count,
                "topics_with_message_count": topic_entries,
                "compression_format": "",
                "compression_mode": "",
                "relative_file_paths": [bag_name],
                "files": [
                    {
                        "path": bag_name,
                        "starting_time": {"nanoseconds_since_epoch": first_ts},
                        "duration": {"nanoseconds": duration_ns},
                        "message_count": message_count,
                    }
                ],
                "custom_data": None,
            }
        }
        info_data = metadata["rosbag2_bagfile_information"]
        logger.debug(
            "Generated synthetic metadata for standalone ROS2 bag %s: version=%s storage=%s "
            "message_count=%s duration_ns=%s relative_files=%s",
            _debug_path_state(bag_path),
            info_data.get("version"),
            info_data.get("storage_identifier"),
            info_data.get("message_count"),
            info_data.get("duration", {}).get("nanoseconds"),
            info_data.get("relative_file_paths"),
        )
        _debug_log_topic_descriptions("Synthetic standalone ROS2 metadata topics", topics)
        return metadata
    finally:
        close_reader = getattr(reader, "close", None)
        if callable(close_reader):
            close_reader()


def _ros2_bag_info_metadata(bag_path: str) -> Dict[str, object]:
    bag = Path(bag_path).resolve()
    if bag.is_dir():
        metadata_path = bag / "metadata.yaml"
        if metadata_path.exists():
            with metadata_path.open("r", encoding="utf-8") as handle:
                metadata = yaml.safe_load(handle) or {}
            if isinstance(metadata, dict):
                return metadata
            raise RuntimeError(
                f"Unexpected rosbag2 metadata format in {metadata_path}: "
                f"expected dict, got {type(metadata).__name__}"
            )
    return _single_file_rosbag2_metadata(str(bag))


def info(bag_path: str) -> Dict[str, object]:
    """Summarize a ROS 2 bag for offline conversion planning."""
    src = Path(bag_path).resolve()
    if not src.exists():
        raise FileNotFoundError(f"Bag not found: {src}")

    metadata = _ros2_bag_info_metadata(str(src))
    bag_info = metadata.get("rosbag2_bagfile_information", {}) if isinstance(metadata, dict) else {}
    storage_id = str(bag_info.get("storage_identifier") or _storage_id_for_bag(str(src)))
    duration_ns = int((bag_info.get("duration") or {}).get("nanoseconds", 0) or 0)
    duration_s = duration_ns / 1e9
    message_count = int(bag_info.get("message_count", 0) or 0)
    size_bytes = bag_size_bytes_raw(src)
    relative_files = list(bag_info.get("relative_file_paths") or [])
    compression_format = str(bag_info.get("compression_format") or "")
    compression_mode = str(bag_info.get("compression_mode") or "")

    topics = []
    for entry in bag_info.get("topics_with_message_count", []) or []:
        topic_metadata = entry.get("topic_metadata", {}) or {}
        topic_name = str(topic_metadata.get("name") or "")
        topic_type = str(topic_metadata.get("type") or "")
        topic_message_count = int(entry.get("message_count", 0) or 0)
        topics.append(
            {
                "name": topic_name,
                "type": topic_type,
                "message_count": topic_message_count,
                "standard_type": _is_standard_ros_msg_type(topic_type),
            }
        )

    topics.sort(key=lambda item: item["name"])
    custom_topics = [t for t in topics if not t["standard_type"]]

    log(f"Bag: {src}")
    log(f"Storage: {storage_id}")
    log(f"Size: {size_bytes} bytes ({_format_bytes(size_bytes)})")
    log(f"Duration: {duration_s:.3f} s")
    log(f"Messages: {message_count}")
    log(f"Topics: {len(topics)}")
    if relative_files:
        log(f"Relative files: {', '.join(relative_files)}")
    if compression_format or compression_mode:
        log(
            "Compression: format=%s mode=%s",
            compression_format or "<none>",
            compression_mode or "<none>",
        )

    for topic in topics:
        logger.info(
            "%-60s %-40s %10d %s",
            topic["name"],
            topic["type"],
            topic["message_count"],
            "" if topic["standard_type"] else "[custom]",
        )

    if custom_topics:
        warn(
            "Custom or non-standard ROS2 topics detected: %s",
            ", ".join(t["name"] for t in custom_topics),
        )
        warn(
            "Start conversion with only standard-message topics or provide matching custom definitions."
        )

    return {
        "path": str(src),
        "storage_identifier": storage_id,
        "size_bytes": size_bytes,
        "duration_seconds": duration_s,
        "message_count": message_count,
        "relative_file_paths": relative_files,
        "compression_format": compression_format,
        "compression_mode": compression_mode,
        "topics": topics,
        "custom_topics": [t["name"] for t in custom_topics],
    }


def _existing_split_member_sources(bag_path: str) -> List[Path]:
    src = Path(bag_path).resolve()
    if not src.is_dir():
        return [src]

    metadata = _ros2_bag_info_metadata(str(src))
    bag_info = metadata.get("rosbag2_bagfile_information", {}) if isinstance(metadata, dict) else {}
    file_entries = list(bag_info.get("files") or [])
    if len(file_entries) <= 1:
        return [src]
    members: List[Path] = []

    for file_entry in file_entries:
        rel_name = str(file_entry.get("path") or "")
        if not rel_name:
            continue
        source_member = src / rel_name
        if not source_member.exists():
            logger.debug("Skipping missing split member %s from %s", source_member, src)
            continue
        members.append(source_member)

    return members or [src]


def _existing_split_member_count(bag_path: str) -> int:
    src = Path(bag_path).resolve()
    if not src.is_dir():
        return 0
    metadata = _ros2_bag_info_metadata(str(src))
    bag_info = metadata.get("rosbag2_bagfile_information", {}) if isinstance(metadata, dict) else {}
    file_entries = list(bag_info.get("files") or [])
    existing = 0
    for file_entry in file_entries:
        rel_name = str(file_entry.get("path") or "")
        if rel_name and (src / rel_name).exists():
            existing += 1
    return existing
