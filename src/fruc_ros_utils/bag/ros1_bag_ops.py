"""Stateless ROS1 bag editing primitives: duration, topic removal, remapping, frame_id, IMU, CameraInfo."""

import logging
import os
import pathlib
import signal
from collections import defaultdict
from typing import Dict, List, Optional

import rosbag
from tqdm import tqdm

from fruc_ros_utils.utils.sensor_conversions import imu_ned_to_enu

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Iteration helpers
# ---------------------------------------------------------------------------

def _iter_bags(paths: List[str], desc: str = "Bags"):
    for bag_path in tqdm(paths, desc=desc, unit="bag"):
        yield bag_path


def _iter_messages(bag, desc: str, topics: Optional[List[str]] = None, raw: bool = False):
    total = bag.get_message_count(topic_filters=topics) if not raw else bag.get_message_count()
    for msg in tqdm(
        bag.read_messages(topics=topics, raw=raw),
        total=total,
        desc=desc,
        unit="msg",
        leave=False,
    ):
        yield msg


def _discover_bags(path: str) -> List[str]:
    p = pathlib.Path(path)
    if p.is_dir():
        return sorted(str(f) for f in p.glob("*.bag*") if f.is_file())
    return [str(p)]


# ---------------------------------------------------------------------------
# Output path helpers
# ---------------------------------------------------------------------------

def _resolve_out_bag(save_bag: str, bag_path: str, multiple: bool) -> str:
    sbp = pathlib.Path(save_bag)
    if sbp.suffix == "" or sbp.is_dir():
        sbp.mkdir(parents=True, exist_ok=True)
        name = f"{pathlib.Path(bag_path).stem}{'_corrected' if multiple else ''}.bag"
        out_path = sbp / name
    else:
        out_path = (
            sbp.parent / f"{pathlib.Path(bag_path).stem}_corrected.bag"
            if multiple else sbp
        )
        out_path.parent.mkdir(parents=True, exist_ok=True)
    logger.debug("Resolved output path: %s", out_path)
    return str(out_path)


def _resolve_remap_out_bag(
    save_bag: Optional[str],
    bag_path: str,
    multiple: bool,
    overwrite: bool,
) -> str:
    bag_path_obj = pathlib.Path(bag_path)
    if overwrite and not save_bag:
        return str(bag_path_obj)
    if save_bag is None:
        if multiple:
            out_dir = bag_path_obj.parent / f"{bag_path_obj.name}_remapped"
            out_dir.mkdir(parents=True, exist_ok=True)
            return str(out_dir / f"{bag_path_obj.stem}_remapped.bag")
        return str(bag_path_obj.with_name(f"{bag_path_obj.stem}_remapped.bag"))
    sbp = pathlib.Path(save_bag)
    if multiple:
        if sbp.suffix and not sbp.is_dir():
            raise ValueError("remap_topics output must be a directory when remapping multiple bags")
        sbp.mkdir(parents=True, exist_ok=True)
        return str(sbp / f"{bag_path_obj.stem}_remapped.bag")
    if sbp.suffix and not sbp.is_dir():
        sbp.parent.mkdir(parents=True, exist_ok=True)
        return str(sbp)
    sbp.mkdir(parents=True, exist_ok=True)
    return str(sbp / f"{bag_path_obj.stem}_remapped.bag")


# ---------------------------------------------------------------------------
# CameraInfo helpers (stubs — functions were referenced but not defined)
# ---------------------------------------------------------------------------

def _discover_camera_info_topics(bag) -> List[str]:
    try:
        info = bag.get_type_and_topic_info()
        topics_info = info.topics if hasattr(info, "topics") else info[1]
    except Exception:
        return []
    return [
        name for name, tinfo in topics_info.items()
        if "CameraInfo" in getattr(tinfo, "msg_type", getattr(tinfo, "datatype", ""))
    ]


def _convert_camera_info_msg(msg):
    return msg


# ---------------------------------------------------------------------------
# Public functions
# ---------------------------------------------------------------------------

def calculate_bag_duration(in_path: str, total: bool = False) -> Dict[str, float]:
    results: Dict[str, float] = {}
    bag_files = _discover_bags(in_path)
    for bag_path in _iter_bags(bag_files, desc="Calculating durations"):
        try:
            with rosbag.Bag(bag_path, "r") as bag:
                duration = bag.get_end_time() - bag.get_start_time()
            results[os.path.basename(bag_path)] = duration
            logger.debug("Duration=%.2fs", duration)
        except Exception as e:
            logger.error("Failed for %s: %s", bag_path, e)
    if total and results:
        total_dur = sum(results.values())
        logger.info("Total duration across %d bags: %.2f s", len(results), total_dur)
        results["__Total__"] = total_dur
    return results


def remove_topic(in_path: str, out_path: str, topics: List[str]) -> None:
    bag_files = _discover_bags(in_path)
    multiple = len(bag_files) > 1
    for bag_file in _iter_bags(bag_files, desc="Removing topics"):
        out_bag_file = _resolve_out_bag(out_path, bag_file, multiple)
        removed_any = False
        with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
            for topic, msg, t in _iter_messages(in_bag, desc=f"Filtering {os.path.basename(bag_file)}"):
                if topic not in topics:
                    out_bag.write(topic, msg, t)
                else:
                    removed_any = True
        if removed_any:
            logger.info("Wrote cleaned bag to %s (removed: %s)", out_bag_file, topics)
        else:
            logger.warning("No matching topics %s found in %s.", topics, bag_file)


def remap_topics(in_path: str, out_path: Optional[str], remap: Dict[str, str], overwrite: bool = False) -> None:
    if not remap:
        raise ValueError("remap_topics requires at least one OLD:NEW mapping")
    bag_files = _discover_bags(in_path)
    multiple = len(bag_files) > 1
    for bag_file in _iter_bags(bag_files, desc="Remapping topics"):
        out_bag_file = _resolve_remap_out_bag(out_path, bag_file, multiple, overwrite)
        out_bag_path = pathlib.Path(out_bag_file)
        if not overwrite and out_bag_path.exists():
            raise FileExistsError(f"Output bag already exists: {out_bag_file}")
        temp_out_path = None
        if overwrite and out_bag_path.resolve() == pathlib.Path(bag_file).resolve():
            temp_out_path = out_bag_path.with_name(f"{out_bag_path.stem}.__remap_tmp__.bag")
            target_path = temp_out_path
        else:
            target_path = out_bag_path
        remapped_messages = 0
        with rosbag.Bag(str(target_path), "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
            for topic, msg, t in _iter_messages(in_bag, desc=f"Remapping {os.path.basename(bag_file)}"):
                new_topic = remap.get(topic, topic)
                if new_topic != topic:
                    remapped_messages += 1
                out_bag.write(new_topic, msg, t)
        if temp_out_path is not None:
            os.replace(str(temp_out_path), str(out_bag_path))
        if remapped_messages:
            logger.info("Wrote remapped bag to %s (%d remapped)", out_bag_file, remapped_messages)
        else:
            logger.warning("No matching remap topics in %s.", bag_file)


def print_topic_sizes(in_path: str) -> Dict[str, Dict[str, int]]:
    results: Dict[str, Dict[str, int]] = {}
    totals: Dict[str, int] = defaultdict(int)
    bag_files = _discover_bags(in_path)
    for bag_file in _iter_bags(bag_files, desc="Topic sizes"):
        sizes: Dict[str, int] = defaultdict(int)
        with rosbag.Bag(bag_file, "r") as bag:
            for topic, msg, _ in _iter_messages(bag, desc=f"Sizes {os.path.basename(bag_file)}", raw=True):
                sizes[topic] += len(msg[1])
                totals[topic] += len(msg[1])
        results[os.path.basename(bag_file)] = dict(sizes)
        for topic, size in sorted(sizes.items(), key=lambda x: x[1]):
            logger.info("%s: %.2f MB", topic, size / 1e6)
    if totals:
        results["__Totals__"] = dict(totals)
    return results


def change_frame_id(in_path: str, out_path: str, topics: List[str], new_frame_id: str) -> None:
    bag_files = _discover_bags(in_path)
    multiple = len(bag_files) > 1
    for bag_file in _iter_bags(bag_files, desc="Changing frame_id"):
        out_bag_file = _resolve_out_bag(out_path, bag_file, multiple) if multiple else out_path
        with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
            for topic, msg, t in _iter_messages(in_bag, desc=f"FrameID {os.path.basename(bag_file)}", topics=topics):
                if hasattr(msg, "header") and hasattr(msg.header, "frame_id") and topic in topics:
                    msg.header.frame_id = new_frame_id
                out_bag.write(topic, msg, t)
        logger.info("Updated frame_id for %s → %s in %s", topics, new_frame_id, out_bag_file)


def convert_imu_to_enu(in_path: str, out_path: str, topics: List[str]) -> None:
    bag_files = _discover_bags(in_path)
    multiple = len(bag_files) > 1
    for bag_file in _iter_bags(bag_files, desc="Converting IMU"):
        out_bag_file = _resolve_out_bag(out_path, bag_file, multiple) if multiple else out_path
        found_topics = {t: False for t in topics}
        with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
            for topic, msg, t in _iter_messages(in_bag, desc=f"IMU {os.path.basename(bag_file)}"):
                if topic in topics and "Imu" in getattr(msg, "_type", ""):
                    found_topics[topic] = True
                    out_bag.write(topic, imu_ned_to_enu(msg), t)
                else:
                    out_bag.write(topic, msg, t)
        for imu_topic, seen in found_topics.items():
            if seen:
                logger.info("Converted %s NED→ENU in %s→%s", imu_topic, bag_file, out_bag_file)
            else:
                logger.warning("IMU topic %s not found in %s.", imu_topic, bag_file)


def convert_camera_info(in_path: str, out_path: str, topics: Optional[List[str]] = None) -> None:
    bag_files = _discover_bags(in_path)
    multiple = len(bag_files) > 1
    for bag_file in _iter_bags(bag_files, desc="Converting CameraInfo"):
        out_bag_file = _resolve_out_bag(out_path, bag_file, multiple) if multiple else out_path
        converted_topics = set()
        with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
            cam_topics = topics or _discover_camera_info_topics(in_bag)
            if not cam_topics:
                logger.warning("No camera_info topics found in %s", bag_file)
            for topic, msg, t in _iter_messages(in_bag, desc=f"CameraInfo {os.path.basename(bag_file)}"):
                if cam_topics and topic in cam_topics:
                    try:
                        mtype = getattr(msg, "_type", "")
                        if mtype and "CameraInfo" not in mtype:
                            logger.warning("Topic %s is not CameraInfo (type=%s). Writing original.", topic, mtype)
                            out_bag.write(topic, msg, t)
                            continue
                        out_bag.write(topic, _convert_camera_info_msg(msg), t)
                        converted_topics.add(topic)
                    except Exception as e:
                        logger.warning("Failed to convert %s: %s. Writing original.", topic, e)
                        out_bag.write(topic, msg, t)
                else:
                    out_bag.write(topic, msg, t)
        if cam_topics:
            missing = [t for t in cam_topics if t not in converted_topics]
            if missing:
                logger.warning("CameraInfo topics not found in %s: %s", bag_file, missing)
        logger.info("Wrote converted bag to %s", out_bag_file)
