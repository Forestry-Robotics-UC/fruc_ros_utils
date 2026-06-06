"""Stateless ROS1 image extraction and manifest round-trip helpers."""

import csv
import logging
import os
import pathlib
import signal
import time
from collections import defaultdict, deque
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge
from tqdm import tqdm

from fruc_ros_utils.bag.ros1_bag_ops import _discover_bags, _iter_bags, _iter_messages
from fruc_ros_utils.utils.image_utils import demosaic_bayer_ros
from fruc_ros_utils.utils.tf_utils import build_R_cam_imu_per_topic, load_extrinsics_yaml

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Type helpers
# ---------------------------------------------------------------------------

def _is_image_datatype(msg_type: str) -> bool:
    return msg_type in (
        "sensor_msgs/Image",
        "sensor_msgs/CompressedImage",
    ) or msg_type.endswith("/Image") or msg_type.endswith("/CompressedImage")


def _discover_image_topics(
    bag: rosbag.Bag, requested_topics: Optional[List[str]]
) -> Tuple[List[str], List[Tuple[str, str]]]:
    try:
        info = bag.get_type_and_topic_info()
        topics_info = info.topics if hasattr(info, "topics") else info[1]
    except Exception:
        topics_info = {}

    requested = set(requested_topics or [])
    image_topics: List[str] = []
    non_image_requested: List[Tuple[str, str]] = []

    for name, tinfo in topics_info.items():
        msg_type = getattr(tinfo, "msg_type", getattr(tinfo, "datatype", ""))
        if not msg_type:
            continue
        if requested and name not in requested:
            continue
        if _is_image_datatype(msg_type):
            image_topics.append(name)
        elif name in requested:
            non_image_requested.append((name, msg_type))

    return image_topics, non_image_requested


def _run_with_timeout(action: Callable, timeout_s: float):
    if timeout_s <= 0.0 or not hasattr(signal, "SIGALRM"):
        return action()

    def _alarm_handler(_signum, _frame):
        raise TimeoutError(f"operation timed out after {timeout_s:.1f}s")

    previous = signal.getsignal(signal.SIGALRM)
    signal.signal(signal.SIGALRM, _alarm_handler)
    signal.setitimer(signal.ITIMER_REAL, timeout_s)
    try:
        return action()
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous)


def _decode_image_to_bgr(msg, bridge: CvBridge):
    msg_type = getattr(msg, "_type", "")
    if msg_type.endswith("CompressedImage"):
        image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    else:
        encoding = getattr(msg, "encoding", "") or ""
        if "bayer" in encoding.lower():
            image = demosaic_bayer_ros(msg)
        else:
            image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    if image is None or getattr(image, "size", 0) == 0:
        raise ValueError("Decoded image is empty")
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.ndim == 3 and image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def _to_secs_nsecs(time_value) -> Tuple[int, int]:
    if hasattr(time_value, "secs") and hasattr(time_value, "nsecs"):
        return int(time_value.secs), int(time_value.nsecs)
    if hasattr(time_value, "to_sec"):
        as_float = float(time_value.to_sec())
    else:
        as_float = float(time_value)
    secs = int(as_float)
    nsecs = int(round((as_float - secs) * 1e9))
    if nsecs >= 1_000_000_000:
        secs += 1
        nsecs -= 1_000_000_000
    return secs, nsecs


def _pick_stamp(msg, bag_time, time_source: str) -> Tuple[int, int, str]:
    header = getattr(msg, "header", None)
    has_header = header is not None and hasattr(header, "stamp")
    if has_header:
        h_secs, h_nsecs = _to_secs_nsecs(header.stamp)
    else:
        h_secs, h_nsecs = 0, 0
    b_secs, b_nsecs = _to_secs_nsecs(bag_time)

    if time_source == "bag":
        return b_secs, b_nsecs, "bag"
    if time_source == "header":
        if has_header and (h_secs != 0 or h_nsecs != 0):
            return h_secs, h_nsecs, "header"
        return b_secs, b_nsecs, "bag_fallback"
    if has_header and (h_secs != 0 or h_nsecs != 0):
        return h_secs, h_nsecs, "header"
    return b_secs, b_nsecs, "bag_fallback"


def _sanitize_topic_for_path(topic: str) -> str:
    cleaned = topic.strip("/")
    if not cleaned:
        return "root"
    return cleaned.replace("/", "__")


def _load_extrinsics(
    bagfiles: List[str],
    topics: List[str],
    imu_cfg: dict,
    extrinsics_yaml: Optional[str] = None,
) -> Dict:
    logger.debug(
        "bagfiles=%s topics=%s imu_cfg=%s extrinsics_yaml=%s",
        bagfiles, topics, imu_cfg, extrinsics_yaml,
    )
    if extrinsics_yaml:
        return {"default": load_extrinsics_yaml(extrinsics_yaml)}
    if imu_cfg.get("imu_frame"):
        R_map = build_R_cam_imu_per_topic(
            sample_bag_path=bagfiles[0],
            topics=topics,
            imu_frame=imu_cfg["imu_frame"],
            urdf_path=imu_cfg.get("urdf_path"),
            tf_static_topic=imu_cfg.get("tf_static_topic", "/tf_static"),
        )
        if not R_map:
            logger.warning(
                "No extrinsics found for imu_frame=%s in urdf=%s",
                imu_cfg["imu_frame"], imu_cfg.get("urdf_path"),
            )
        return R_map
    return {}


# ---------------------------------------------------------------------------
# Public functions
# ---------------------------------------------------------------------------

def extract_images(
    in_path: str,
    topics: List[str],
    with_context: bool = False,
    ctx_size: int = 3,
    topic_discovery_timeout_s: float = 5.0,
) -> Dict[str, List]:
    results: Dict[str, List] = {}
    bag_files = _discover_bags(in_path)

    for bag_file in bag_files:
        bridge = CvBridge()
        images = []
        ctx = deque(maxlen=ctx_size)

        with rosbag.Bag(bag_file, "r") as bag:
            if topics:
                image_topics = list(topics)
                non_image_requested: List[Tuple[str, str]] = []
            else:
                try:
                    image_topics, non_image_requested = _run_with_timeout(
                        lambda: _discover_image_topics(bag, topics),
                        timeout_s=topic_discovery_timeout_s,
                    )
                except TimeoutError:
                    logger.warning(
                        "Topic discovery timed out after %.1fs for %s. "
                        "Pass --topics to skip discovery and start immediately.",
                        topic_discovery_timeout_s, bag_file,
                    )
                    results[os.path.basename(bag_file)] = images
                    continue

            for name, msg_type in non_image_requested:
                logger.warning(
                    "Topic requested but not an image: %s (type=%s) in %s",
                    name, msg_type, bag_file,
                )

            if not image_topics:
                logger.warning("No image topics found (requested=%s) in %s", topics, bag_file)
                results[os.path.basename(bag_file)] = images
                continue

            for topic, msg, t in _iter_messages(
                bag,
                desc=f"Images {os.path.basename(bag_file)}",
                topics=image_topics,
            ):
                try:
                    cv_img = _decode_image_to_bgr(msg, bridge)
                    ts = t.to_sec()
                    if with_context:
                        ctx.append(cv_img)
                        images.append((ts, cv_img, list(ctx)))
                    else:
                        images.append((ts, cv_img))
                except Exception as e:
                    logger.warning(
                        "Failed to decode image at %.6f on %s in %s: %s",
                        t.to_sec(), topic, bag_file, e,
                    )

        results[os.path.basename(bag_file)] = images
        logger.debug("%s → %d frames", bag_file, len(images))

    return results


def extract_images_manifest(
    in_path: str,
    out_dir: str,
    topics: Optional[List[str]] = None,
    manifest_name: str = "image_manifest",
    manifest_format: str = "csv",
    time_source: str = "auto",
    topic_discovery_timeout_s: float = 5.0,
    startup_timeout_s: float = 25.0,
) -> Dict[str, int]:
    bag_files = _discover_bags(in_path)
    if not bag_files:
        raise ValueError(f"No bag files found in {in_path}")

    out_root = pathlib.Path(out_dir)
    out_root.mkdir(parents=True, exist_ok=True)

    manifest_stem = pathlib.Path(manifest_name).stem if pathlib.Path(manifest_name).suffix else manifest_name
    if not manifest_stem:
        manifest_stem = "image_manifest"

    manifest_specs: List[Tuple[pathlib.Path, str]] = []
    if manifest_format in ("csv", "both"):
        manifest_specs.append((out_root / f"{manifest_stem}.csv", ","))
    if manifest_format in ("txt", "both"):
        manifest_specs.append((out_root / f"{manifest_stem}.txt", "\t"))
    if not manifest_specs:
        raise ValueError(f"Unsupported manifest format: {manifest_format}")

    fieldnames = [
        "bag_path", "bag_name", "topic", "message_index",
        "stamp_source", "stamp_sec", "stamp_nsec", "stamp",
        "bag_time_sec", "bag_time_nsec", "bag_time",
        "frame_id", "seq", "encoding", "width", "height", "step",
        "is_bigendian", "image_relpath", "image_path",
    ]

    handles: List = []
    writers: List[csv.DictWriter] = []
    written_paths: List[pathlib.Path] = []
    saved_images = 0

    try:
        for path, delimiter in manifest_specs:
            path.parent.mkdir(parents=True, exist_ok=True)
            handle = path.open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=fieldnames, delimiter=delimiter)
            writer.writeheader()
            handles.append(handle)
            writers.append(writer)
            written_paths.append(path)

        for bag_index, bag_file in enumerate(bag_files, start=1):
            bag_name = os.path.basename(bag_file)
            bag_stem = pathlib.Path(bag_name).stem
            frame_idx_per_topic: Dict[str, int] = defaultdict(int)
            logger.info("Extracting bag %d/%d: %s", bag_index, len(bag_files), bag_name)
            startup_t0 = time.monotonic()

            def _remaining_startup_timeout(preferred_timeout: float) -> float:
                if preferred_timeout <= 0.0:
                    return 0.0
                if startup_timeout_s <= 0.0:
                    return preferred_timeout
                elapsed = time.monotonic() - startup_t0
                remaining = startup_timeout_s - elapsed
                if remaining <= 0.0:
                    return 0.0
                return min(preferred_timeout, remaining)

            open_timeout = _remaining_startup_timeout(startup_timeout_s if startup_timeout_s > 0.0 else 0.0)
            if startup_timeout_s > 0.0 and open_timeout <= 0.0:
                logger.warning(
                    "Startup timeout reached before opening %s (timeout=%.1fs); skipping bag",
                    bag_file, startup_timeout_s,
                )
                continue

            def _open_bag():
                try:
                    return rosbag.Bag(bag_file, "r", allow_unindexed=True, skip_index=True)
                except TypeError:
                    return rosbag.Bag(bag_file, "r")

            try:
                bag = _run_with_timeout(_open_bag, timeout_s=open_timeout)
            except TimeoutError:
                logger.warning(
                    "Opening bag timed out after %.1fs for %s; skipping bag",
                    open_timeout, bag_file,
                )
                continue

            with bag:
                if topics:
                    image_topics = list(topics)
                else:
                    discover_timeout = _remaining_startup_timeout(topic_discovery_timeout_s)
                    if topic_discovery_timeout_s > 0.0 and discover_timeout <= 0.0:
                        logger.warning(
                            "Startup timeout reached before topic discovery in %s "
                            "(timeout=%.1fs); skipping bag",
                            bag_file, startup_timeout_s,
                        )
                        continue
                    try:
                        image_topics, non_image_requested = _run_with_timeout(
                            lambda: _discover_image_topics(bag, topics),
                            timeout_s=discover_timeout,
                        )
                    except TimeoutError:
                        logger.warning(
                            "Topic discovery timed out after %.1fs for %s. "
                            "Provide --topics to skip discovery and start immediately.",
                            topic_discovery_timeout_s, bag_file,
                        )
                        continue
                    for name, msg_type in non_image_requested:
                        logger.warning(
                            "Topic requested but not an image: %s (type=%s) in %s",
                            name, msg_type, bag_file,
                        )
                if not image_topics:
                    logger.warning("No image topics found (requested=%s) in %s", topics, bag_file)
                    continue

                bridge = CvBridge()
                saved_in_bag = 0
                topic_label = image_topics[0] if len(image_topics) == 1 else f"{len(image_topics)} topics"
                fallback_partial_scan = startup_timeout_s > 0.0
                if fallback_partial_scan:
                    logger.info(
                        "Startup timeout mode enabled (%.1fs): starting partial sequential scan "
                        "and filtering selected topics=%s",
                        startup_timeout_s, image_topics,
                    )
                    iterator_source = bag.read_messages()
                else:
                    iterator_source = bag.read_messages(topics=image_topics)

                iterator_desc = (
                    f"Scanning all msgs ({bag_index}/{len(bag_files)})"
                    if fallback_partial_scan
                    else f"Reading {topic_label} ({bag_index}/{len(bag_files)})"
                )
                iterator = tqdm(
                    iterator_source, desc=iterator_desc,
                    unit="msg", leave=True, dynamic_ncols=True,
                )
                scanned_in_bag = 0
                first_image_seen = False
                first_image_timeout_warned = False
                scan_t0 = time.monotonic()

                for topic, msg, bag_time in iterator:
                    scanned_in_bag += 1
                    if fallback_partial_scan and topic not in image_topics:
                        if scanned_in_bag % 5000 == 0:
                            iterator.set_postfix(scanned=scanned_in_bag, saved=saved_in_bag)
                        if (
                            startup_timeout_s > 0.0
                            and not first_image_timeout_warned
                            and (time.monotonic() - scan_t0) >= startup_timeout_s
                        ):
                            logger.warning(
                                "No image message found within %.1fs in %s for topics=%s; "
                                "continuing partial scan",
                                startup_timeout_s, bag_file, image_topics,
                            )
                            first_image_timeout_warned = True
                        continue
                    first_image_seen = True
                    try:
                        cv_img = _decode_image_to_bgr(msg, bridge)
                    except Exception as exc:
                        logger.warning(
                            "Failed to decode image at %.6f on %s in %s: %s",
                            bag_time.to_sec(), topic, bag_file, exc,
                        )
                        continue

                    stamp_sec, stamp_nsec, stamp_source_label = _pick_stamp(msg, bag_time, time_source)
                    bag_sec, bag_nsec = _to_secs_nsecs(bag_time)

                    frame_idx_per_topic[topic] += 1
                    msg_index = frame_idx_per_topic[topic]

                    topic_dir = out_root / bag_stem / _sanitize_topic_for_path(topic)
                    topic_dir.mkdir(parents=True, exist_ok=True)
                    image_name = f"{stamp_sec:010d}_{stamp_nsec:09d}_{msg_index:06d}.png"
                    image_path = topic_dir / image_name

                    if not cv2.imwrite(str(image_path), cv_img):
                        logger.warning("Failed to write PNG %s", image_path)
                        continue

                    image_relpath = image_path.relative_to(out_root).as_posix()
                    header = getattr(msg, "header", None)
                    row = {
                        "bag_path": str(pathlib.Path(bag_file).resolve()),
                        "bag_name": bag_name,
                        "topic": topic,
                        "message_index": msg_index,
                        "stamp_source": stamp_source_label,
                        "stamp_sec": stamp_sec,
                        "stamp_nsec": stamp_nsec,
                        "stamp": f"{stamp_sec}.{stamp_nsec:09d}",
                        "bag_time_sec": bag_sec,
                        "bag_time_nsec": bag_nsec,
                        "bag_time": f"{bag_sec}.{bag_nsec:09d}",
                        "frame_id": getattr(header, "frame_id", ""),
                        "seq": getattr(header, "seq", msg_index),
                        "encoding": getattr(msg, "encoding", "bgr8"),
                        "width": getattr(msg, "width", cv_img.shape[1]),
                        "height": getattr(msg, "height", cv_img.shape[0]),
                        "step": getattr(msg, "step", cv_img.shape[1] * 3),
                        "is_bigendian": getattr(msg, "is_bigendian", 0),
                        "image_relpath": image_relpath,
                        "image_path": str(image_path.resolve()),
                    }
                    for writer in writers:
                        writer.writerow(row)
                    saved_images += 1
                    saved_in_bag += 1
                    if saved_in_bag % 200 == 0:
                        if fallback_partial_scan:
                            iterator.set_postfix(scanned=scanned_in_bag, saved=saved_in_bag, topic=topic)
                        else:
                            iterator.set_postfix(saved=saved_in_bag, topic=topic)

                if not first_image_seen:
                    logger.warning(
                        "No messages observed for selected image topics=%s in %s",
                        image_topics, bag_file,
                    )
                logger.info("Saved %d image(s) from %s", saved_in_bag, bag_name)
    finally:
        for handle in handles:
            handle.close()

    if saved_images == 0:
        logger.warning("No images were extracted from %s", in_path)
    else:
        logger.info("Extracted %d images into %s", saved_images, out_root)
        for manifest_path in written_paths:
            logger.info("Wrote manifest: %s", manifest_path)

    return {
        "bags": len(bag_files),
        "images": saved_images,
        "manifests": len(written_paths),
    }


def images_manifest_to_bag(
    manifest_path: str,
    out_bag: str,
    images_root: Optional[str] = None,
    topic_override: Optional[str] = None,
    frame_id_override: Optional[str] = None,
    output_encoding: str = "bgr8",
    write_time: str = "stamp",
    delimiter_mode: str = "auto",
    strict: bool = False,
) -> Dict[str, int]:
    manifest = pathlib.Path(manifest_path)
    if not manifest.exists():
        raise FileNotFoundError(f"Manifest not found: {manifest_path}")

    root = pathlib.Path(images_root) if images_root else manifest.parent
    out_bag_path = pathlib.Path(out_bag)
    out_bag_path.parent.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()

    delimiter_lookup = {"comma": ",", "tab": "\t", "semicolon": ";"}

    def _parse_int(row: Dict[str, str], key: str, default: int = 0) -> int:
        raw = row.get(key)
        if raw is None or raw == "":
            return default
        try:
            return int(float(raw))
        except Exception:
            return default

    with manifest.open("r", newline="", encoding="utf-8") as handle:
        if delimiter_mode == "auto":
            sample = handle.read(4096)
            handle.seek(0)
            try:
                dialect = csv.Sniffer().sniff(sample, delimiters=",\t;")
                delimiter = dialect.delimiter
            except Exception:
                delimiter = "\t" if manifest.suffix.lower() == ".txt" else ","
        else:
            delimiter = delimiter_lookup[delimiter_mode]

        reader = csv.DictReader(handle, delimiter=delimiter)
        fieldnames = set(reader.fieldnames or [])
        if not {"image_relpath", "image_path"} & fieldnames:
            raise ValueError("Manifest must include 'image_relpath' or 'image_path' column")

        written = 0
        skipped = 0
        with rosbag.Bag(str(out_bag_path), "w") as bag:
            for row in tqdm(reader, desc="Rebuilding image bag", unit="img"):
                image_key = row.get("image_relpath") or row.get("image_path") or ""
                if not image_key:
                    skipped += 1
                    if strict:
                        raise ValueError("Manifest row missing image_relpath/image_path")
                    logger.warning("Skipping row without image path column")
                    continue

                image_path = pathlib.Path(image_key)
                if not image_path.is_absolute():
                    image_path = root / image_path
                image_path = image_path.resolve()
                if not image_path.exists():
                    skipped += 1
                    if strict:
                        raise FileNotFoundError(f"Image path not found: {image_path}")
                    logger.warning("Image not found, skipping: %s", image_path)
                    continue

                cv_img = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
                if cv_img is None:
                    skipped += 1
                    if strict:
                        raise ValueError(f"Failed to read image: {image_path}")
                    logger.warning("Failed to read image, skipping: %s", image_path)
                    continue

                if output_encoding in ("bgr8", "rgb8"):
                    if cv_img.ndim == 2:
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
                    elif cv_img.ndim == 3 and cv_img.shape[2] == 4:
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
                    if output_encoding == "rgb8":
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                elif output_encoding == "mono8" and cv_img.ndim == 3:
                    if cv_img.shape[2] == 4:
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

                stamp_sec = _parse_int(row, "stamp_sec")
                stamp_nsec = _parse_int(row, "stamp_nsec")
                if stamp_sec == 0 and stamp_nsec == 0 and row.get("stamp"):
                    stamp_sec, stamp_nsec = _to_secs_nsecs(float(row["stamp"]))
                stamp_time = rospy.Time(stamp_sec, stamp_nsec)

                bag_sec = _parse_int(row, "bag_time_sec")
                bag_nsec = _parse_int(row, "bag_time_nsec")
                if bag_sec == 0 and bag_nsec == 0 and row.get("bag_time"):
                    bag_sec, bag_nsec = _to_secs_nsecs(float(row["bag_time"]))
                bag_time = rospy.Time(bag_sec, bag_nsec)

                topic = topic_override or row.get("topic") or "/camera/image_raw"
                msg = bridge.cv2_to_imgmsg(cv_img, encoding=output_encoding)
                msg.header.stamp = stamp_time
                msg.header.frame_id = (
                    frame_id_override if frame_id_override is not None else row.get("frame_id", "")
                )
                msg.header.seq = _parse_int(row, "seq", written + 1)

                write_stamp = bag_time if write_time == "bag" and (bag_sec or bag_nsec) else stamp_time
                bag.write(topic, msg, write_stamp)
                written += 1

    logger.info("Wrote %d images to %s (skipped=%d)", written, out_bag_path, skipped)
    return {"written": written, "skipped": skipped}
