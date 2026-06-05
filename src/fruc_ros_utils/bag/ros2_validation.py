"""Validation helpers for ROS 2 → ROS 1 bag conversion."""

import logging
from collections import defaultdict
from typing import Dict, List, Optional

from fruc_ros_utils.bag._ros2_helpers import (
    _debug_path_state,
    _debug_topic_count_summary,
    _is_standard_ros_msg_type,
    _make_progress_bar,
    _OPTIONAL_DROPPED_ROS2_TYPES,
)
from fruc_ros_utils.bag.ros2_inspector import open_ros2_reader
from fruc_ros_utils.bag.ros2_topic_filter import _source_topic_types
from fruc_ros_utils.bag.ouster_conv_helpers import _ouster_packets_per_scan, _pick_ouster_points_topic
from fruc_ros_utils.utils.logging_utils import get_logger

logger = get_logger("Ros2utils", level="INFO", log_file=None)
log = logger.info
warn = logger.warning
err = logger.error


def _warn_custom_types(bag_path: str) -> None:
    try:
        reader = open_ros2_reader(bag_path)
        topics = reader.get_all_topics_and_types()
        for t in topics:
            if not _is_standard_ros_msg_type(t.type):
                warn(
                    f"Custom type '{t.type}' on topic '{t.name}' is not in "
                    f"the standard ROS1 typestore — it will be excluded from "
                    f"ROS1 conversion unless a custom bridge registers an "
                    f"explicit ROS1 definition and md5sum for it."
                )
    except Exception as e:
        warn(f"Could not pre-check types: {e}")


def _validate_timestamps(bag_path: str, max_drift_s: float = 1.0) -> None:
    try:
        import rosbag
    except ImportError:
        return

    SAMPLE = 5

    try:
        logger.debug(
            "Starting ROS1 timestamp validation for %s max_drift_s=%s sample_size=%s",
            _debug_path_state(bag_path),
            max_drift_s,
            SAMPLE,
        )
        with rosbag.Bag(bag_path, "r") as bag:
            topic_map: Dict[str, List[float]] = {}
            for topic, msg, t in bag.read_messages():
                drifts = topic_map.setdefault(topic, [])
                if len(drifts) >= SAMPLE:
                    continue
                if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                    header_s = msg.header.stamp.to_sec()
                    bag_s = t.to_sec()
                    drifts.append(abs(header_s - bag_s))

            for topic, drifts in topic_map.items():
                if not drifts:
                    continue
                mean_drift = sum(drifts) / len(drifts)
                logger.debug(
                    "Timestamp drift samples for %s: %s",
                    topic,
                    ", ".join(f"{value:.6f}" for value in drifts),
                )
                if mean_drift > max_drift_s:
                    warn(
                        f"TIMESTAMP DRIFT on '{topic}': mean |bag_time - "
                        f"header.stamp| = {mean_drift:.3f}s  "
                        f"(threshold {max_drift_s}s).  "
                        f"Adjust downstream time windows relative to header.stamp, not bag recording time."
                    )
                else:
                    log(f"  ✓ {topic}: timestamp drift {mean_drift:.4f}s ok")
    except Exception as e:
        warn(f"Timestamp validation skipped: {e}")


def _count_ros2_messages_per_topic(bag_path: str) -> Dict[str, int]:
    counts: Dict[str, int] = defaultdict(int)
    logger.debug("Counting ROS2 messages per topic for %s", _debug_path_state(bag_path))
    reader = open_ros2_reader(bag_path)
    pbar = _make_progress_bar("Counting ROS2 source messages", "msg")
    try:
        while reader.has_next():
            topic_name, _, _ = reader.read_next()
            counts[topic_name] += 1
            if pbar is not None:
                pbar.update(1)
    finally:
        if pbar is not None:
            pbar.close()
    counts_dict = dict(counts)
    logger.debug("ROS2 message counts summary: %s", _debug_topic_count_summary(counts_dict))
    return counts_dict


def _count_ros1_messages_per_topic(bag_path: str) -> Dict[str, int]:
    counts: Dict[str, int] = defaultdict(int)
    try:
        from rosbags.rosbag1 import Reader as Ros1Reader
        logger.debug(
            "Counting ROS1 messages per topic with rosbags.rosbag1.Reader for %s",
            _debug_path_state(bag_path),
        )
        pbar = _make_progress_bar("Counting ROS1 output messages", "msg")
        try:
            with Ros1Reader(bag_path) as reader:
                for conn, _, _ in reader.messages():
                    counts[conn.topic] += 1
                    if pbar is not None:
                        pbar.update(1)
        finally:
            if pbar is not None:
                pbar.close()
        counts_dict = dict(counts)
        logger.debug(
            "ROS1 message counts summary via rosbags reader: %s",
            _debug_topic_count_summary(counts_dict),
        )
        return counts_dict
    except Exception:
        logger.debug(
            "rosbags.rosbag1.Reader counting failed for %s; retrying with rosbag",
            _debug_path_state(bag_path),
            exc_info=True,
        )

    try:
        import rosbag
        logger.debug(
            "Counting ROS1 messages per topic with rosbag.Bag for %s",
            _debug_path_state(bag_path),
        )
        pbar = _make_progress_bar("Counting ROS1 output messages", "msg")
        try:
            with rosbag.Bag(bag_path, "r") as bag:
                for topic, _, _ in bag.read_messages():
                    counts[topic] += 1
                    if pbar is not None:
                        pbar.update(1)
        finally:
            if pbar is not None:
                pbar.close()
        counts_dict = dict(counts)
        logger.debug(
            "ROS1 message counts summary via rosbag: %s",
            _debug_topic_count_summary(counts_dict),
        )
        return counts_dict
    except Exception as e:
        raise RuntimeError(f"Unable to count messages in ROS1 bag '{bag_path}': {e}")


def _expected_topic_counts(
    src_counts: Dict[str, int],
    include_topics: Optional[List[str]],
    exclude_topics: Optional[List[str]],
    remap: Optional[Dict[str, str]],
) -> Dict[str, int]:
    include_set = set(include_topics or [])
    exclude_set = set(exclude_topics or [])
    expected: Dict[str, int] = defaultdict(int)
    for src_topic, count in src_counts.items():
        if include_set and src_topic not in include_set:
            continue
        if src_topic in exclude_set:
            continue
        dst_topic = remap.get(src_topic, src_topic) if remap else src_topic
        expected[dst_topic] += int(count)
    return dict(expected)


def _validate_ouster_packet_topic(
    src_path: str,
    packet_topic: str,
    packet_count: int,
    dst_counts: Dict[str, int],
) -> bool:
    prefix = packet_topic[: -len("/lidar_packets")]
    points_topic = _pick_ouster_points_topic(dst_counts, prefix)
    if points_topic is None:
        err(
            "Missing %s and no corresponding point topic found under prefix '%s'",
            packet_topic,
            prefix,
        )
        return False

    actual_scans = int(dst_counts.get(points_topic, 0))
    if actual_scans <= 0:
        err(
            "Point topic %s has zero messages while %s had %d packets",
            points_topic,
            packet_topic,
            packet_count,
        )
        return False

    ouster_info = _ouster_packets_per_scan(src_path, prefix)
    packets_per_scan = ouster_info.get("packets_per_scan")
    metadata_topic = ouster_info.get("metadata_topic", f"{prefix}/metadata")
    metadata_found = bool(ouster_info.get("metadata_found"))
    model = ouster_info.get("model")
    channel_count = ouster_info.get("channel_count")
    lidar_mode = ouster_info.get("lidar_mode")
    cols = ouster_info.get("columns_per_frame")
    cols_packet = ouster_info.get("columns_per_packet")

    if not metadata_found:
        err(
            "Missing required Ouster metadata topic %s for %s. "
            "Packet->scan validation must use metadata.",
            metadata_topic,
            packet_topic,
        )
        return False

    if not packets_per_scan:
        err(
            "Could not infer packets/scan for %s from %s "
            "(model=%s, channels=%s, lidar_mode=%s, columns_per_frame=%s, columns_per_packet=%s).",
            packet_topic,
            metadata_topic,
            model,
            channel_count,
            lidar_mode,
            cols,
            cols_packet,
        )
        return False

    expected_scans = float(packet_count) / float(packets_per_scan)
    tolerance = max(2.0, min(50.0, expected_scans * 0.001))
    delta = abs(float(actual_scans) - expected_scans)
    log(
        "Ouster packet->scan check: %s packets=%d -> %s scans=%d "
        "(model=%s, channels=%s, lidar_mode=%s, columns_per_frame=%s, "
        "columns_per_packet=%s, packets_per_scan=%d, expected≈%.2f, delta=%.2f, tolerance=%.2f)",
        packet_topic,
        packet_count,
        points_topic,
        actual_scans,
        model or "unknown",
        channel_count if channel_count is not None else "unknown",
        lidar_mode or "unknown",
        cols if cols is not None else "unknown",
        cols_packet if cols_packet is not None else "unknown",
        packets_per_scan,
        expected_scans,
        delta,
        tolerance,
    )
    if delta <= tolerance:
        return True

    err(
        "Packet->scan validation failed for %s -> %s (expected≈%.2f scans, got=%d, tolerance=%.2f)",
        packet_topic,
        points_topic,
        expected_scans,
        actual_scans,
        tolerance,
    )
    return False


def _validate_message_counts(
    src_path: str,
    dst_path: str,
    include_topics: Optional[List[str]] = None,
    exclude_topics: Optional[List[str]] = None,
    remap: Optional[Dict[str, str]] = None,
) -> None:
    log("Validating per-topic message counts (source ROS2 vs output ROS1)...")
    logger.debug(
        "Message-count validation inputs: src=%s dst=%s include=%s exclude=%s remap=%s",
        _debug_path_state(src_path),
        _debug_path_state(dst_path),
        include_topics,
        exclude_topics,
        remap,
    )
    src_counts = _count_ros2_messages_per_topic(src_path)
    source_types = _source_topic_types(src_path)
    expected_counts = _expected_topic_counts(src_counts, include_topics, exclude_topics, remap)
    dst_counts = _count_ros1_messages_per_topic(dst_path)
    logger.debug("Expected output count summary: %s", _debug_topic_count_summary(expected_counts))
    logger.debug("Actual ROS1 output count summary: %s", _debug_topic_count_summary(dst_counts))

    mismatches: List[str] = []
    missing: List[str] = []
    extras: List[str] = []
    soft_missing: List[str] = []

    for topic, expected in sorted(expected_counts.items()):
        actual = int(dst_counts.get(topic, 0))
        source_type = source_types.get(topic)
        if actual == 0:
            if source_type in _OPTIONAL_DROPPED_ROS2_TYPES:
                soft_missing.append(f"{topic} (type={source_type}, expected={expected})")
                logger.debug(
                    "Allowing dropped optional topic: topic=%s type=%s expected=%s",
                    topic,
                    source_type,
                    expected,
                )
                continue
            if topic.endswith("/imu_packets"):
                soft_missing.append(f"{topic} (imu packets, expected={expected})")
                logger.debug(
                    "Allowing dropped imu packet topic: topic=%s expected=%s", topic, expected
                )
                continue
            if topic.endswith("/lidar_packets"):
                if _validate_ouster_packet_topic(src_path, topic, expected, dst_counts):
                    soft_missing.append(f"{topic} (validated via packet->scan metadata check)")
                    logger.debug(
                        "Allowing lidar packet topic after packet->scan validation: topic=%s expected=%s",
                        topic,
                        expected,
                    )
                    continue
            missing.append(topic)
            logger.debug(
                "Missing output topic: topic=%s type=%s expected=%s actual=%s",
                topic,
                source_type,
                expected,
                actual,
            )
        elif actual != expected:
            mismatches.append(f"{topic}: expected={expected} actual={actual}")
            logger.debug(
                "Output topic count mismatch: topic=%s type=%s expected=%s actual=%s",
                topic,
                source_type,
                expected,
                actual,
            )

    for topic in sorted(dst_counts.keys()):
        if topic not in expected_counts:
            extras.append(f"{topic}: actual={dst_counts[topic]}")
            logger.debug("Extra output topic: topic=%s actual=%s", topic, dst_counts[topic])

    if soft_missing:
        warn("Allowed missing topics after conversion:")
        for line in soft_missing:
            warn(f"  {line}")

    if missing or mismatches:
        for topic in missing:
            err(f"Missing topic after conversion: {topic}")
        for line in mismatches:
            err(f"Message count mismatch: {line}")
        if extras:
            warn("Extra topics in output bag (not part of expected filtered set):")
            for line in extras:
                warn(f"  {line}")
        details: List[str] = []
        if missing:
            details.append("missing=" + ", ".join(missing[:10]))
        if mismatches:
            details.append("mismatches=" + "; ".join(mismatches[:10]))
        logger.debug(
            "Message-count validation failure details: missing=%s mismatches=%s extras=%s source_types=%s",
            missing,
            mismatches,
            extras,
            {
                topic: source_types.get(topic)
                for topic in sorted(
                    set(missing) | {m.split(":", 1)[0] for m in mismatches}
                )
            },
        )
        raise RuntimeError(
            "Message-count validation failed; conversion output is incomplete or inconsistent. "
            + " | ".join(details)
        )

    if extras:
        warn("Output contains extra topics outside expected filtered set:")
        for line in extras:
            warn(f"  {line}")

    log(f"Message-count validation passed for {len(expected_counts)} topic(s).")
