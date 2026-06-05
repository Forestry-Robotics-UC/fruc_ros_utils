"""Topic filtering and selection helpers for ROS 2 bags."""

import logging
from pathlib import Path
from typing import Dict, List, Optional, Union

from fruc_ros_utils.bag._ros2_helpers import (
    _debug_path_state,
    _FAILED_PARSE_MSGTYPE_RE,
    _is_ffmpeg_packet_msg_type,
    _is_standard_ros_msg_type,
)
from fruc_ros_utils.bag.ros2_inspector import open_ros2_reader
from fruc_ros_utils.utils.logging_utils import get_logger

logger = get_logger("Ros2utils", level="INFO", log_file=None)


def _topic_selected(
    topic_name: str,
    include_topics: Optional[List[str]] = None,
    exclude_topics: Optional[List[str]] = None,
) -> bool:
    include_set = set(include_topics or [])
    exclude_set = set(exclude_topics or [])
    if include_set and topic_name not in include_set:
        return False
    if topic_name in exclude_set:
        return False
    return True


def _source_topic_types(bag_path: str) -> Dict[str, str]:
    reader = open_ros2_reader(bag_path)
    try:
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        logger.debug(
            "Source ROS2 topic types for %s: %s",
            _debug_path_state(bag_path),
            topic_types,
        )
        return topic_types
    finally:
        close_reader = getattr(reader, "close", None)
        if callable(close_reader):
            close_reader()


def _selected_topic_names(
    bag_path: Union[str, Path],
    include_topics: Optional[List[str]] = None,
    exclude_topics: Optional[List[str]] = None,
) -> List[str]:
    include_set = set(include_topics or [])
    exclude_set = set(exclude_topics or [])
    selected = []
    for topic_name in sorted(_source_topic_types(str(bag_path)).keys()):
        if include_set and topic_name not in include_set:
            continue
        if topic_name in exclude_set:
            continue
        selected.append(topic_name)
    logger.debug(
        "Selected topics in %s with include=%s exclude=%s: %s",
        _debug_path_state(bag_path),
        include_topics,
        exclude_topics,
        selected,
    )
    return selected


def _topics_for_msg_type(
    bag_path: Union[str, Path],
    msg_type: str,
    include_topics: Optional[List[str]] = None,
    exclude_topics: Optional[List[str]] = None,
) -> List[str]:
    include_set = set(include_topics or [])
    exclude_set = set(exclude_topics or [])
    topic_types = _source_topic_types(str(bag_path))
    topics = []
    for topic_name, current_type in sorted(topic_types.items()):
        if include_set and topic_name not in include_set:
            continue
        if topic_name in exclude_set:
            continue
        if current_type == msg_type:
            topics.append(topic_name)
    logger.debug(
        "Topics matching type %s in %s with include=%s exclude=%s: %s",
        msg_type,
        _debug_path_state(bag_path),
        include_topics,
        exclude_topics,
        topics,
    )
    return topics


def _extract_parse_failed_msgtype(details: str) -> Optional[str]:
    match = _FAILED_PARSE_MSGTYPE_RE.search(str(details))
    if not match:
        return None
    return match.group(1)


def _auto_exclude_unsupported_topics(
    bag_path: str,
    include_topics: Optional[List[str]] = None,
    exclude_topics: Optional[List[str]] = None,
) -> List[str]:
    include_set = set(include_topics or [])
    exclude_set = set(exclude_topics or [])
    auto_excluded: List[str] = []
    for topic_name, msg_type in _source_topic_types(bag_path).items():
        if include_set and topic_name not in include_set:
            continue
        if topic_name in exclude_set:
            continue
        if _is_standard_ros_msg_type(msg_type):
            continue
        auto_excluded.append(topic_name)
    return sorted(auto_excluded)


def _ffmpeg_decoded_output_topic(
    input_topic: str,
    existing_topics: Optional[set] = None,
    used_topics: Optional[set] = None,
) -> str:
    if input_topic.endswith("/ffmpeg"):
        base_topic = input_topic[: -len("/ffmpeg")]
    else:
        base_topic = f"{input_topic}/decoded"

    existing_topics = existing_topics or set()
    used_topics = used_topics or set()
    if base_topic not in existing_topics and base_topic not in used_topics:
        return base_topic

    candidate = f"{base_topic}_decoded"
    suffix = 1
    while candidate in existing_topics or candidate in used_topics:
        suffix += 1
        candidate = f"{base_topic}_decoded_{suffix}"
    return candidate


def _discover_ffmpeg_decode_topic_map(
    topic_types: Dict[str, str],
    include_topics: Optional[List[str]] = None,
    exclude_topics: Optional[List[str]] = None,
) -> Dict[str, str]:
    include_set = set(include_topics or [])
    exclude_set = set(exclude_topics or [])
    existing_topics = set(topic_types.keys())
    used_outputs: set = set()
    topic_map: Dict[str, str] = {}

    for topic_name, msg_type in sorted(topic_types.items()):
        if not _is_ffmpeg_packet_msg_type(msg_type):
            continue
        out_topic = _ffmpeg_decoded_output_topic(
            topic_name,
            existing_topics=existing_topics,
            used_topics=used_outputs,
        )
        if topic_name in exclude_set or out_topic in exclude_set:
            continue
        if include_set and topic_name not in include_set and out_topic not in include_set:
            continue
        topic_map[topic_name] = out_topic
        used_outputs.add(out_topic)

    return topic_map
