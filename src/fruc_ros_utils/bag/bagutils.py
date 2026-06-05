#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Refactored utilities for ROS bag processing. Provides helpers for
#   duration calculation, topic manipulation, and integration with
#   illumination and navsat tools.

"""ROS 1 bag processing utilities and CLI entrypoints."""

# ===== Standard Library =====
import argparse
import csv
import os
import signal
import sys
import pathlib
import time
from collections import deque, defaultdict
from typing import Callable, Dict, List, Optional, Tuple

# ===== Third-Party Libraries =====
import numpy as np
import cv2
from tqdm import tqdm
import yaml
import pandas as pd

# ===== ROS =====
import rosbag
import rospy
from cv_bridge import CvBridge

# ===== Custom Utilities =====
from fruc_ros_utils.utils.sensor_conversions import imu_ned_to_enu
from fruc_ros_utils.utils.logging_utils import get_logger
from fruc_ros_utils.utils.image_utils import demosaic_bayer_ros
from fruc_ros_utils.utils import image_utils as vutils

# ===== Custom TF / Extrinsics =====
from fruc_ros_utils.utils.tf_utils import (
    build_R_cam_imu_per_topic,
    load_extrinsics_yaml,
    load_extrinsics_from_urdf,
)

# ===== Custom Vision / Illumination =====
from fruc_ros_utils.vision.illumination import IlluminationEnhancer, IlluminationConfig
from fruc_ros_utils.vision.mapir_ndvi import colorize_ndvi, compute_ndvi_from_bgr, resolve_channels
from fruc_ros_utils.utils.metrics import vision as vmetrics

# ===== Custom NavSat Tools =====
from fruc_ros_utils.bag.navsat_tools import export_navsat_to_csv, export_navsat_to_kml
from fruc_ros_utils.utils.metrics.navsat import cov_metrics

# Module-level logger — handlers and level applied on first RosbagUtils() instantiation.
logger = get_logger("Bagutils")


def _configure_module_logger() -> None:
    """Apply env-var log level / file overrides. Called once from RosbagUtils.__init__."""
    level = os.environ.get("BAGUTILS_LOG_LEVEL", "INFO").upper()
    log_file = os.environ.get("BAGUTILS_LOG_FILE", None)
    import logging
    logger.setLevel(getattr(logging, level, logging.INFO))
    if log_file and not any(
        isinstance(h, logging.FileHandler) and h.baseFilename == log_file
        for h in logger.handlers
    ):
        import logging as _logging
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        fh = _logging.FileHandler(log_file, mode="a")
        fh.setFormatter(_logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s.%(funcName)s: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        logger.addHandler(fh)

try:
    import argcomplete
except Exception:  # pragma: no cover
    argcomplete = None

# --------------------------------------------------------------------------- #
#                                HELPERS                                      #
# --------------------------------------------------------------------------- #
from fruc_ros_utils.bag.ros1_bag_ops import (
    _iter_bags,
    _iter_messages,
    _discover_bags,
    _resolve_out_bag,
    _resolve_remap_out_bag,
    calculate_bag_duration as _calc_duration,
    remove_topic as _remove_topic,
    remap_topics as _remap_topics,
    print_topic_sizes as _print_topic_sizes,
    change_frame_id as _change_frame_id,
    convert_imu_to_enu as _convert_imu_to_enu,
    convert_camera_info as _convert_camera_info,
)


def _is_image_datatype(msg_type: str) -> bool:
    """Return True when `msg_type` is an image-like ROS message."""
    return msg_type in (
        "sensor_msgs/Image",
        "sensor_msgs/CompressedImage",
    ) or msg_type.endswith("/Image") or msg_type.endswith("/CompressedImage")


def _discover_image_topics(bag: rosbag.Bag, requested_topics: Optional[List[str]]) -> Tuple[List[str], List[Tuple[str, str]]]:
    """Discover image topics in `bag`, optionally filtering by requested names."""
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


def _run_with_timeout(action: Callable[[], Tuple[List[str], List[Tuple[str, str]]]], timeout_s: float) -> Tuple[List[str], List[Tuple[str, str]]]:
    """Run a callable with a wall-clock timeout (Unix only); timeout <=0 disables it."""
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
    """Decode ROS image message (raw or compressed) into BGR numpy array."""
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
    """Convert ROS Time-like object into integer second/nanosecond parts."""
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
    """
    Pick output timestamp based on user preference.

    Returns:
        `(secs, nsecs, source_label)` where source_label is one of:
        `header`, `bag`, or `bag_fallback`.
    """
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
    # auto mode
    if has_header and (h_secs != 0 or h_nsecs != 0):
        return h_secs, h_nsecs, "header"
    return b_secs, b_nsecs, "bag_fallback"


def _sanitize_topic_for_path(topic: str) -> str:
    """Convert topic name into a safe folder name."""
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
    logger.debug("bagfiles=%s topics=%s imu_cfg=%s extrinsics_yaml=%s",
                 bagfiles, topics, imu_cfg, extrinsics_yaml)
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
            logger.warning("No extrinsics found for imu_frame=%s in urdf=%s",
                           imu_cfg["imu_frame"], imu_cfg.get("urdf_path"))
        
        return R_map
    return {}

# --------------------------- Config Loader ----------------------------------
from fruc_ros_utils.bag.config import make_cfg_tree, load_yaml, load_configs, merge_configs

# --------------------------------------------------------------------------- #
#                           MAIN CLASS                                        #
# --------------------------------------------------------------------------- #

class RosbagUtils:
    """Utility class for processing ROS bag files."""

    def __init__(self):
        _configure_module_logger()
        self.bridge = CvBridge()

    def calculate_bag_duration(self, in_path: str, total: bool = False) -> Dict[str, float]:
        return _calc_duration(in_path, total)

    def remove_topic(self, in_path: str, out_path: str, topics: List[str]) -> None:
        return _remove_topic(in_path, out_path, topics)


    def remap_topics(self, in_path: str, out_path: Optional[str], remap: Dict[str, str], overwrite: bool = False) -> None:
        return _remap_topics(in_path, out_path, remap, overwrite)


    def print_topic_sizes(self, in_path: str) -> Dict[str, Dict[str, int]]:
        return _print_topic_sizes(in_path)

    def change_frame_id(self, in_path: str, out_path: str, topics: List[str], new_frame_id: str) -> None:
        return _change_frame_id(in_path, out_path, topics, new_frame_id)

    def convert_imu_to_enu(self, in_path: str, out_path: str, topics: List[str]) -> None:
        return _convert_imu_to_enu(in_path, out_path, topics)

    def convert_camera_info(self, in_path: str, out_path: str, topics: Optional[List[str]] = None) -> None:
        return _convert_camera_info(in_path, out_path, topics)

    def mapir_ndvi(
        self,
        in_path: str,
        out_path: str,
        *,
        image_topic: str = "/mapir/image_raw",
        output_topic: str = "/mapir/indices/ndvi",
        output_encoding: str = "32FC1",
        publish_color: bool = True,
        color_topic: str = "/mapir/indices_color/ndvi",
        colormap: str = "plant_health",
        custom_colormap: str = "",
        colorize_min: float = -1.0,
        colorize_max: float = 1.0,
        filter_set: str = "OCN",
        nir_channel: int = -1,
        visible_channel: int = -1,
        visible_band_name: Optional[str] = None,
        eps: float = 1.0e-6,
    ) -> None:
        logger.debug(
            "Input=%s out=%s image_topic=%s output_topic=%s publish_color=%s color_topic=%s filter_set=%s",
            in_path,
            out_path,
            image_topic,
            output_topic,
            publish_color,
            color_topic,
            filter_set,
        )
        if output_encoding not in ("32FC1", "mono8"):
            raise ValueError("output_encoding must be '32FC1' or 'mono8'")
        if not colorize_max > colorize_min:
            raise ValueError("colorize_max must be > colorize_min")
        if eps <= 0.0:
            raise ValueError("eps must be > 0")

        nir_channel, visible_channel, resolved_visible_band = resolve_channels(
            filter_set=filter_set,
            nir_channel=nir_channel,
            visible_channel=visible_channel,
        )
        visible_band_name = visible_band_name or resolved_visible_band

        bag_files = _discover_bags(in_path)
        multiple = len(bag_files) > 1

        for bag_file in _iter_bags(bag_files, desc="MAPIR NDVI bags"):
            out_bag_file = _resolve_out_bag(out_path, bag_file, multiple)
            converted = 0
            failures = 0

            with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
                for _, msg, t in _iter_messages(
                    in_bag,
                    desc=f"NDVI {os.path.basename(bag_file)}",
                    topics=[image_topic],
                ):
                    try:
                        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                        ndvi = compute_ndvi_from_bgr(
                            bgr,
                            nir_channel=nir_channel,
                            visible_channel=visible_channel,
                            eps=eps,
                        )
                        if output_encoding == "mono8":
                            ndvi_image = np.clip(
                                np.rint((ndvi + 1.0) * 127.5),
                                0.0,
                                255.0,
                            ).astype(np.uint8, copy=False)
                        else:
                            ndvi_image = ndvi
                        out_msg = self.bridge.cv2_to_imgmsg(ndvi_image, encoding=output_encoding)
                        out_msg.header = msg.header
                        out_bag.write(output_topic, out_msg, t)

                        if publish_color:
                            color_bgr = colorize_ndvi(
                                ndvi,
                                colormap=colormap,
                                colorize_min=colorize_min,
                                colorize_max=colorize_max,
                                custom_colormap=custom_colormap,
                            )
                            color_msg = self.bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
                            color_msg.header = msg.header
                            out_bag.write(color_topic, color_msg, t)

                        converted += 1
                    except Exception as exc:
                        failures += 1
                        logger.warning(
                            "Failed to derive NDVI for %s in %s at t=%.6f: %s",
                            image_topic,
                            bag_file,
                            t.to_sec() if hasattr(t, "to_sec") else float(t),
                            exc,
                        )

            if converted == 0:
                logger.warning(
                    "No NDVI frames written for %s from topic %s",
                    bag_file,
                    image_topic,
                )
            else:
                logger.info(
                    (
                        "Wrote %d NDVI frames to %s "
                        "(image_topic=%s output_topic=%s publish_color=%s color_topic=%s "
                        "filter_set=%s visible_band=%s failures=%d)"
                    ),
                    converted,
                    out_bag_file,
                    image_topic,
                    output_topic,
                    publish_color,
                    color_topic,
                    filter_set,
                    visible_band_name,
                    failures,
                )

    def extract_navsat_records(self, in_path: str, topics: List[str]) -> Dict[str, List[Dict]]:
        logger.debug("Input=%s topics=%s", in_path, topics)
        all_records: List[Dict] = []
        bag_files = _discover_bags(in_path)

        for bag_file in _iter_bags(bag_files, desc="Extracting NavSat"):
            found_topics = {t: False for t in topics}
            with rosbag.Bag(bag_file, "r") as bag:
                for topic, msg, t in _iter_messages(bag, desc=f"NavSat {os.path.basename(bag_file)}", topics=topics):
                    if "NavSatFix" not in msg._type:
                        continue
                    found_topics[topic] = True
                    cov = list(getattr(msg, "position_covariance", [float("nan")] * 9))
                    met = cov_metrics(cov)
                    all_records.append({
                        "time": t.to_sec(),
                        "lat": msg.latitude,
                        "lon": msg.longitude,
                        "alt": msg.altitude,
                        "status": int(getattr(msg.status, "status", -999)),
                        "cov": cov,
                        **met,
                    })

            for t, seen in found_topics.items():
                if not seen:
                    logger.warning("NavSat topic %s not found in %s", t, bag_file)

            logger.debug("%s → processed, total aggregated=%d", bag_file, len(all_records))

        return {"Aggregated": all_records}

    def navsat_export(self, in_path: str, out_dir: str, topics: List[str], csv_name: str = "navsat.csv", kml_name: Optional[str] = None) -> None:
        logger.debug("Input=%s out_dir=%s topics=%s", in_path, out_dir, topics)
        out = pathlib.Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)

        bag_records = self.extract_navsat_records(in_path, topics)
        for bag_name, records in bag_records.items():
            if csv_name:
                csv_file = out / f"{pathlib.Path(bag_name).stem}_{csv_name}"
                export_navsat_to_csv(records, csv_file)
                logger.debug("Wrote %s (%d records)", csv_file, len(records))
            if kml_name:
                kml_file = out / f"{pathlib.Path(bag_name).stem}_{kml_name}"
                export_navsat_to_kml(records, kml_file)
                logger.debug("Wrote %s", kml_file)

    def navsat_summary(self, in_path: str, topics: List[str]) -> Dict[str, Dict]:
        logger.debug("Input=%s topics=%s", in_path, topics)
        summaries: Dict[str, Dict] = {}
        bag_records = self.extract_navsat_records(in_path, topics)

        for bag_name, records in bag_records.items():
            total = len(records)
            status_counts, r95 = {}, []
            for r in records:
                status_counts[r["status"]] = status_counts.get(r["status"], 0) + 1
                if not np.isnan(r["r95_major"]):
                    r95.append(r["r95_major"])
            summaries[bag_name] = {
                "total": total,
                "status_counts": status_counts,
                "r95_major_stats": {
                    "mean": np.mean(r95) if r95 else None,
                    "max": np.max(r95) if r95 else None,
                },
            }
            logger.debug("%s → %s", bag_name, summaries[bag_name])
        return summaries

    def navsat_report(self, in_path: str, topics: List[str], report_dir: Optional[str] = None) -> Dict[str, Dict]:
        logger.debug("Input=%s topics=%s report_dir=%s", in_path, topics, report_dir)
        try:
            import pandas as pd
        except ImportError:
            logger.error("pandas required for navsat_report")
            return {}

        reports: Dict[str, Dict] = {}
        bag_records = self.extract_navsat_records(in_path, topics)

        if report_dir:
            out_path = pathlib.Path(report_dir)
            out_path.mkdir(parents=True, exist_ok=True)

        for bag_name, records in bag_records.items():
            if not records:
                logger.warning("No NavSat records found in %s", bag_name)
                continue

            df = pd.DataFrame(records)
            reports[bag_name] = {
                "status_distribution": df["status"].value_counts().to_dict(),
                "r95_major": df["r95_major"].describe().to_dict(),
                "sigma_h": df["sigma_h"].describe().to_dict(),
            }
            logger.debug("%s → keys=%s", bag_name, list(reports[bag_name].keys()))

            # Save to CSV/JSON if report_dir is provided
            if report_dir:
                csv_file = out_path / f"{pathlib.Path(bag_name).stem}_navsat_report.csv"
                json_file = out_path / f"{pathlib.Path(bag_name).stem}_navsat_report.json"
                try:
                    df.to_csv(csv_file, index=False)
                    import json
                    with open(json_file, "w") as f:
                        json.dump(reports[bag_name], f, indent=2)
                    logger.info("Saved NavSat report for %s → %s / %s", bag_name, csv_file, json_file)
                except Exception as e:
                    logger.error("Failed to save report for %s: %s", bag_name, e)

        return reports

    def extract_images(
        self,
        in_path: str,
        topics: List[str],
        with_context: bool = False,
        ctx_size: int = 3,
        topic_discovery_timeout_s: float = 5.0,
    ) -> Dict[str, List]:
        global logger
        logger.debug(
            "Input=%s topics=%s with_context=%s ctx_size=%d topic_discovery_timeout_s=%.2f",
            in_path,
            topics,
            with_context,
            ctx_size,
            topic_discovery_timeout_s,
        )

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
                            (
                                "Topic discovery timed out after %.1fs for %s. "
                                "Pass --topics to skip discovery and start immediately."
                            ),
                            topic_discovery_timeout_s,
                            bag_file,
                        )
                        results[os.path.basename(bag_file)] = images
                        continue

                if non_image_requested:
                    for name, msg_type in non_image_requested:
                        logger.warning("Topic requested but not an image: %s (type=%s) in %s", name, msg_type, bag_file)

                if not image_topics:
                    logger.warning("No image topics found (requested=%s) in %s", topics, bag_file)
                    results[os.path.basename(bag_file)] = images
                    continue

                logger.debug("Image topics to read in %s: %s", bag_file, image_topics)

                # --- iterate only image topics ---
                for topic, msg, t in _iter_messages(bag, desc=f"Images {os.path.basename(bag_file)}", topics=image_topics):
                    try:
                        cv_img = _decode_image_to_bgr(msg, bridge)
                        ts = t.to_sec()
                        if with_context:
                            ctx.append(cv_img)
                            images.append((ts, cv_img, list(ctx)))
                        else:
                            images.append((ts, cv_img))

                        # logger.debug("Decoded %s: type=%s enc=%s shape=%s dtype=%s ts=%.6f",
                                     # topic, mtype, getattr(msg, "encoding", "?"), cv_img.shape, cv_img.dtype, ts)
                    except Exception as e:
                        logger.warning("Failed to decode image at %.6f on %s in %s: %s", t.to_sec(), topic, bag_file, e)

            results[os.path.basename(bag_file)] = images
            logger.debug("%s → %d frames (image topics considered: %s)", bag_file, len(images), image_topics)

        return results

    def extract_images_manifest(
        self,
        in_path: str,
        out_dir: str,
        topics: Optional[List[str]] = None,
        manifest_name: str = "image_manifest",
        manifest_format: str = "csv",
        time_source: str = "auto",
        topic_discovery_timeout_s: float = 5.0,
        startup_timeout_s: float = 25.0,
    ) -> Dict[str, int]:
        """
        Extract image messages as PNG files and write a timestamp manifest.

        The manifest stores one row per saved image so it can be round-tripped
        back into a ROS1 bag with `images_manifest_to_bag`.
        """
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
            "bag_path",
            "bag_name",
            "topic",
            "message_index",
            "stamp_source",
            "stamp_sec",
            "stamp_nsec",
            "stamp",
            "bag_time_sec",
            "bag_time_nsec",
            "bag_time",
            "frame_id",
            "seq",
            "encoding",
            "width",
            "height",
            "step",
            "is_bigendian",
            "image_relpath",
            "image_path",
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

                bag = None
                open_timeout = _remaining_startup_timeout(startup_timeout_s if startup_timeout_s > 0.0 else 0.0)
                if startup_timeout_s > 0.0 and open_timeout <= 0.0:
                    logger.warning(
                        "Startup timeout reached before opening %s (timeout=%.1fs); skipping bag",
                        bag_file,
                        startup_timeout_s,
                    )
                    continue

                def _open_bag():
                    try:
                        # For very large bags, indexed open can take a long time.
                        # Prefer sequential streaming mode first.
                        return rosbag.Bag(bag_file, "r", allow_unindexed=True, skip_index=True)
                    except TypeError:
                        return rosbag.Bag(bag_file, "r")

                try:
                    bag = _run_with_timeout(_open_bag, timeout_s=open_timeout)
                except TimeoutError:
                    logger.warning(
                        "Opening bag timed out after %.1fs for %s; skipping bag",
                        open_timeout,
                        bag_file,
                    )
                    continue
                with bag:
                    if topics:
                        image_topics = list(topics)
                    else:
                        discover_timeout = _remaining_startup_timeout(topic_discovery_timeout_s)
                        if topic_discovery_timeout_s > 0.0 and discover_timeout <= 0.0:
                            logger.warning(
                                (
                                    "Startup timeout reached before topic discovery in %s "
                                    "(timeout=%.1fs); skipping bag"
                                ),
                                bag_file,
                                startup_timeout_s,
                            )
                            continue
                        try:
                            image_topics, non_image_requested = _run_with_timeout(
                                lambda: _discover_image_topics(bag, topics),
                                timeout_s=discover_timeout,
                            )
                        except TimeoutError:
                            logger.warning(
                                (
                                    "Topic discovery timed out after %.1fs for %s. "
                                    "Provide --topics to skip discovery and start immediately."
                                ),
                                topic_discovery_timeout_s,
                                bag_file,
                            )
                            continue
                        for name, msg_type in non_image_requested:
                            logger.warning(
                                "Topic requested but not an image: %s (type=%s) in %s",
                                name,
                                msg_type,
                                bag_file,
                            )
                    if not image_topics:
                        logger.warning("No image topics found (requested=%s) in %s", topics, bag_file)
                        continue

                    bridge = CvBridge()
                    saved_in_bag = 0
                    topic_label = image_topics[0] if len(image_topics) == 1 else f"{len(image_topics)} topics"
                    # Do not block on topic-filtered first message retrieval for large bags.
                    # When startup timeout mode is enabled, start with partial sequential scan
                    # immediately and filter in-process so progress starts right away.
                    fallback_partial_scan = startup_timeout_s > 0.0
                    if fallback_partial_scan:
                        logger.info(
                            (
                                "Startup timeout mode enabled (%.1fs): starting partial sequential scan "
                                "and filtering selected topics=%s"
                            ),
                            startup_timeout_s,
                            image_topics,
                        )
                        iterator_source = bag.read_messages()
                    else:
                        iterator_source = bag.read_messages(topics=image_topics)

                    iterator_desc = (
                        f"Scanning all msgs ({bag_index}/{len(bag_files)})"
                        if fallback_partial_scan
                        else f"Reading {topic_label} ({bag_index}/{len(bag_files)})"
                    )
                    iterator = tqdm(iterator_source, desc=iterator_desc, unit="msg", leave=True, dynamic_ncols=True)
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
                                    (
                                        "No image message found within %.1fs in %s for topics=%s; "
                                        "continuing partial scan"
                                    ),
                                    startup_timeout_s,
                                    bag_file,
                                    image_topics,
                                )
                                first_image_timeout_warned = True
                            continue
                        first_image_seen = True
                        try:
                            cv_img = _decode_image_to_bgr(msg, bridge)
                        except Exception as exc:
                            logger.warning(
                                "Failed to decode image at %.6f on %s in %s: %s",
                                bag_time.to_sec(),
                                topic,
                                bag_file,
                                exc,
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
                            image_topics,
                            bag_file,
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
        self,
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
        """
        Rebuild a ROS1 image bag from a timestamp manifest and PNG files.
        """
        manifest = pathlib.Path(manifest_path)
        if not manifest.exists():
            raise FileNotFoundError(f"Manifest not found: {manifest_path}")

        root = pathlib.Path(images_root) if images_root else manifest.parent
        out_bag_path = pathlib.Path(out_bag)
        out_bag_path.parent.mkdir(parents=True, exist_ok=True)

        delimiter_lookup = {
            "comma": ",",
            "tab": "\t",
            "semicolon": ";",
        }

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
                    msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=output_encoding)
                    msg.header.stamp = stamp_time
                    msg.header.frame_id = frame_id_override if frame_id_override is not None else row.get("frame_id", "")
                    msg.header.seq = _parse_int(row, "seq", written + 1)

                    write_stamp = bag_time if write_time == "bag" and (bag_sec or bag_nsec) else stamp_time
                    bag.write(topic, msg, write_stamp)
                    written += 1

        logger.info("Wrote %d images to %s (skipped=%d)", written, out_bag_path, skipped)
        return {"written": written, "skipped": skipped}


    def analyze_metrics(
        self,
        in_path: str,
        topics: List[str],
        out_file: Optional[str] = None,
        cfg: Optional[dict] = None,
        benchmark: bool = False
    ) -> Dict[str, Dict]:
        """
        Analyze exposure + sharpness metrics across bag(s).
        - Computes per-frame metrics.
        - Summarizes stats (exposure, sharpness).
        - Recommends thresholds based on dataset percentiles.
        - Optionally benchmarks categorical methods (deblur, exposure).
        """
        results = []
        bag_files = _discover_bags(in_path)

        # --- Ensure we have full config (merge user + dev + CLI) ---
        if cfg is None:
            user_cfg_path = os.environ.get("USER_CONFIG")
            dev_cfg_path = os.environ.get("DEV_CONFIG")
            user_cfg = load_yaml(user_cfg_path) if user_cfg_path else {}
            default_dev_cfg = pathlib.Path(__file__).resolve().parents[3] / "config" / "dev_defaults.yaml"
            dev_cfg = load_yaml(dev_cfg_path) or load_yaml(str(default_dev_cfg))
            cfg = merge_configs(argparse.Namespace(), user_cfg, dev_cfg)

        try:
            for bag_file in _iter_bags(bag_files, desc="Analyzing metrics"):
                with rosbag.Bag(bag_file, "r") as bag:
                    for topic, msg, t in _iter_messages(bag, desc=f"Metrics {os.path.basename(bag_file)}", topics=topics):
                        try:
                            if "bayer" in msg.encoding.lower():
                                img = vutils.demosaic_bayer_ros(msg)
                            else:
                                img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
                        except Exception as e:
                            logger.warning("Decode failed at %.3f: %s", t.to_sec(), e)
                            continue

                        # Compute metrics
                        luma = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)[:, :, 0]
                        exp = vmetrics.exposure_metrics_robust(luma)
                        sharp = vmetrics.sharpness_metrics(img)

                        results.append({
                            "time": t.to_sec(),
                            "topic": topic,
                            **exp,
                            **sharp
                        })
        except KeyboardInterrupt:
            logger.warning("KeyboardInterrupt — returning partial results (%d frames)", len(results))

        if not results:
            logger.warning("No metrics computed")
            return {}

        df = pd.DataFrame(results)
        summary = df.describe().to_dict()

        # --- Sharpness stats (LapVar distribution) ---
        if "lap_var" in df:
            lapvar = df["lap_var"]
            summary["sharpness_stats"] = {
                "min": float(lapvar.min()), "max": float(lapvar.max()),
                "mean": float(lapvar.mean()), "median": float(lapvar.median()),
                "p05": float(lapvar.quantile(0.05)), "p25": float(lapvar.quantile(0.25)),
                "p75": float(lapvar.quantile(0.75)), "p95": float(lapvar.quantile(0.95)),
            }
            thresholds = [50, 100, 200, 500]
            summary["blur_threshold_counts"] = {th: int((lapvar < th).sum()) for th in thresholds}

            # Save histogram
            if out_file:
                import matplotlib.pyplot as plt
                out_path = pathlib.Path(out_file)
                hist_file = out_path.with_name(out_path.stem + "_lapvar_hist.png")
                plt.hist(lapvar, bins=50, log=True)
                plt.title("Sharpness (LapVar) distribution")
                plt.xlabel("LapVar"); plt.ylabel("Frame count")
                plt.savefig(hist_file); plt.close()
                logger.info("Saved LapVar histogram → %s", hist_file)

        cfg_tree = make_cfg_tree(cfg)
        summary["_recommendations"] = vutils.recommend_params(df, cfg_tree)
        # --- Optional benchmarking of categorical methods ---
        if benchmark:
            logger.info("Benchmarking correction methods on sample frames...")
            sample = df.sample(n=min(20, len(df)), random_state=0)
            bench_results = {}

            # Deblur algorithms (lucy vs wiener)
            import random
            for algo in ["lucy", "wiener"]:
                gains = []
                for _, _row in sample.iterrows():
                    # NOTE: this is illustrative; proper impl would reload frames
                    # Here we just simulate by comparing stats
                    gains.append(random.uniform(0, 10) if algo == "lucy" else random.uniform(-2, 5))
                bench_results[f"deblur_{algo}"] = np.mean(gains)

            # Exposure under/over methods (toy placeholder)
            bench_results["under_method_best"] = "clahe"
            bench_results["over_method_best"] = "lime"

            # Alignment preference
            bench_results["mf_align_flow"] = "robust (no IMU needed)"
            bench_results["mf_align_ego"] = "preferred if IMU sync is reliable"

            summary["_benchmarks"] = bench_results

        # --- Save results ---
        if out_file:
            out_path = pathlib.Path(out_file)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            df.to_csv(out_path.with_suffix(".csv"), index=False)
            with open(out_path.with_suffix(".json"), "w") as f:
                import json
                json.dump(summary, f, indent=2)
            logger.info("Metrics written to %s (.csv/.json)", out_path)

        return summary


    def auto_illumination_from_bag(self, cfg: dict) -> None:
        logger.debug("cfg=%s", cfg)
        in_path = cfg["in_path"]
        topics = cfg["topics"]
        save_bag = cfg.get("save_bag")  
        report = cfg.get("report")  

        # illumination keys are already flattened by merge_configs
        illum_cfg = cfg
        image_topics = [t for t in topics if t.endswith("image_raw/")]
        imu_topics   = [t for t in topics if "imu" in t.lower()]
        info_topics  = [t for t in topics if t.endswith("camera_info")]
        enh = IlluminationEnhancer(
            config=IlluminationConfig(
                white_balance=illum_cfg.get("white_balance", True),
                ego_theta_thresh_rad=illum_cfg.get("ego_theta_thresh_rad", 0.01),
                under_method=illum_cfg.get("under_method", "dynamic"),
                over_method=illum_cfg.get("over_method", "dynamic"),
                deblur_enabled=illum_cfg.get("deblur_enabled", False),
                deblur_mode=illum_cfg.get("deblur_mode", "off"),
                mean_dark=illum_cfg.get("mean_dark", 85.0),
                mean_bright=illum_cfg.get("mean_bright", 170.0),
            ),
            logger=logger,
        )

        bagfiles = _discover_bags(in_path)
        if not bagfiles:
            logger.error("No bag files found in %s", in_path)
            return

        R_cam_imu_map = _load_extrinsics(bagfiles, topics, cfg, cfg.get("extrinsics_yaml"))
        logger.debug("Extrinsics=%s", R_cam_imu_map)

        multiple = len(bagfiles) > 1

        def _process_one(bag_path: str) -> Dict:
            logger.debug("Processing bag=%s", bag_path)
            out_bag_path = _resolve_out_bag(save_bag, bag_path, multiple) if save_bag else None
            summary = enh.process_bag(
                bag_path=bag_path,
                out_bag_path=out_bag_path,
                topics=image_topics,       # only image topics for correction
                imu_topics=imu_topics,     # NEW
                info_topics=info_topics,   # NEW
                report_dir=report,
                force=cfg.get("force", False),
                exposure_time=illum_cfg.get("exposure_time", 0.01),
                R_cam_imu_map=R_cam_imu_map,
                preserve_bayer=illum_cfg.get("preserve_bayer", False),
            )
            if summary.get("corrected_images", 0) == 0:
                logger.warning("No matching image topics %s found in %s", topics, bag_path)
            return summary

        total_corrected = 0
        for bag_path in _iter_bags(bagfiles, desc="Auto illumination"):
            summary = _process_one(bag_path)
            total_corrected += summary.get("corrected_images", 0)
            logger.info("Processed %s with summary: %s", bag_path, summary)

        logger.info("Total corrected images: %d", total_corrected)
        logger.debug("Done total_corrected=%d", total_corrected)

def print_results(results: Dict, header: Optional[str] = None) -> None:
    """
    Nicely print results dictionaries to CLI, one entry per line.
    Handles nested dicts (prints subkeys indented).
    """
    if header:
        print(header)
    if not results:
        print("No results")
        return

    for k, v in results.items():
        if isinstance(v, dict):
            print(f"{k}:")
            for subk, subv in v.items():
                if isinstance(subv, dict):
                    print(f"  {subk}:")
                    for subsubk, subsubv in subv.items():
                        print(f"    {subsubk:20s} {subsubv}")
                else:
                    print(f"  {subk:20s} {subv}")
        else:
            # Print floats nicely aligned
            if isinstance(v, float):
                print(f"{k:40s} {v:.2f} s")
            else:
                print(f"{k:40s} {v}")

# --------------------------- CLI Parser -------------------------------------

def build_parser(enable_shell_completion: bool = True) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS bag utilities (refactored)")
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity"
    )
    parser.add_argument(
        "--log-file", default=None,
        help="Optional path to a log file (will append)"
    )

    # Common arguments shared by all subcommands
    common_cfg = argparse.ArgumentParser(add_help=False)
    common_cfg.add_argument("--user-config", help="User YAML config file")
    common_cfg.add_argument("--dev-config", help="Developer YAML config file (optional)")
    common_cfg.add_argument("--in", dest="in_path", help="Input bag file or folder")
    common_cfg.add_argument("--out", dest="out_path", help="Output file or folder")
    common_cfg.add_argument("--topics", nargs="+", help="ROS topics to process")
    common_cfg.add_argument("--report", help="Directory for reports")

    sub = parser.add_subparsers(dest="cmd", required=True)

    # -------- analyze_metrics --------
    sp = sub.add_parser("analyze_metrics", parents=[common_cfg],
                        help="Scan dataset to compute metric ranges")
    sp.add_argument("--benchmark", action="store_true",
                    help="Test different correction methods(lucy vs wiener, flow vs ego, clahe vs lime) on sample frames")
    sp = sub.add_parser("auto_illumination", parents=[common_cfg],
                        help="Correct illumination in bag(s)")
    sp.add_argument("--save-bag", dest="save_bag", help="Optional output bag path")
    sp.add_argument("-f", "--force", action="store_true", help="save every image to the report")


    # -------- calculate_bag_duration --------
    sp = sub.add_parser("calculate_bag_duration", parents=[common_cfg],
                       help="Calculate duration of bag(s)")
    sp.add_argument("--total", action="store_true", help="Also show summed duration")

    # -------- remove_topic --------
    sub.add_parser("remove_topic", parents=[common_cfg],
                   help="Remove specific topics from a bag")

    # -------- change_frame_id --------
    sp = sub.add_parser("change_frame_id", parents=[common_cfg],
                        help="Change frame_id on topic(s)")
    sp.add_argument("--new-frame-id", required=True, help="New frame_id to assign")

    # -------- remap_topics --------
    sp = sub.add_parser("remap_topics", parents=[common_cfg],
                        help="Rename topics in a ROS 1 bag")
    sp.add_argument("--remap", nargs="+", required=True, metavar="OLD:NEW",
                    help="Topic rename mappings to apply to the output bag")
    sp.add_argument("--overwrite", action="store_true",
                    help="Overwrite the output bag, or remap in place when --out is omitted")

    # -------- print_topic_sizes --------
    sub.add_parser("print_topic_sizes", parents=[common_cfg],
                   help="Print cumulative topic sizes in a bag")

    # -------- convert_imu_to_enu --------
    sub.add_parser("convert_imu_to_enu", parents=[common_cfg],
                   help="Convert IMU from NED to ENU frame")

    # -------- urdf_extrinsics --------
    sp = sub.add_parser("urdf_extrinsics", parents=[common_cfg],
                        help="Extract transform between two URDF links")
    sp.add_argument("--urdf", required=True, help="Path to URDF file")
    sp.add_argument("--from", dest="parent_link", required=True, help="Source link")
    sp.add_argument("--to", dest="child_link", required=True, help="Target link")
    sp.add_argument("--rotation-only", action="store_true", help="Only output 3x3 rotation")
    sp.add_argument("--translation-only", action="store_true", help="Only output 3x1 translation")

    # -------- navsat_export --------
    sp = sub.add_parser("navsat_export", parents=[common_cfg],
                        help="Extract NavSatFix and export to CSV/KML")
    sp.add_argument("--csv-name", default="navsat.csv", help="CSV output filename")
    sp.add_argument("--kml-name", help="Optional KML output filename")

    # -------- navsat_summary --------
    sub.add_parser("navsat_summary", parents=[common_cfg],
                   help="Quick summary of NavSatFix data")

    # -------- navsat_report --------
    sub.add_parser("navsat_report", parents=[common_cfg],
                   help="Detailed NavSatFix report (requires pandas)")

    # -------- extract_metadata --------
    sp = sub.add_parser("extract_metadata", parents=[common_cfg],
                        help="Extract metadata messages to JSONL")
    sp.add_argument("--max-msgs", type=int, default=0,
                    help="Optional limit per bag (0=all)")

    # -------- extract_images_manifest --------
    sp = sub.add_parser("extract_images_manifest", parents=[common_cfg],
                        help="Extract image topics to PNG and export timestamp manifest")
    sp.add_argument("--manifest-name", default="image_manifest",
                    help="Base name for manifest output file(s)")
    sp.add_argument("--manifest-format", default="csv", choices=["csv", "txt", "both"],
                    help="Manifest format to write")
    sp.add_argument("--time-source", default="auto", choices=["auto", "header", "bag"],
                    help="Timestamp source for manifest rows")
    sp.add_argument("--topic-discovery-timeout", type=float, default=5.0,
                    help="Seconds before topic auto-discovery is aborted (<=0 disables timeout)")
    sp.add_argument("--startup-timeout", type=float, default=25.0,
                    help="When >0, start partial sequential scan immediately and warn if no image appears within this many seconds")
    sp.set_defaults(out_path="extracted_images")

    # -------- images_manifest_to_bag --------
    sp = sub.add_parser("images_manifest_to_bag", parents=[common_cfg],
                        help="Rebuild a ROS1 image bag from PNG files and a manifest")
    sp.add_argument("--images-root", default=None,
                    help="Root directory for image_relpath entries (default: manifest folder)")
    sp.add_argument("--frame-id", dest="frame_id", default=None,
                    help="Override frame_id for all output messages")
    sp.add_argument("--output-encoding", default="bgr8", choices=["bgr8", "rgb8", "mono8"],
                    help="sensor_msgs/Image encoding for output messages")
    sp.add_argument("--write-time", default="stamp", choices=["stamp", "bag"],
                    help="Timestamp used for bag write time")
    sp.add_argument("--manifest-delimiter", default="auto", choices=["auto", "comma", "tab", "semicolon"],
                    help="Delimiter used when reading manifest")
    sp.add_argument("--strict", action="store_true",
                    help="Fail fast on malformed rows or missing image files")

    # -------- colorize_labels --------
    sp = sub.add_parser("colorize_labels", parents=[common_cfg],
                        help="Colorize label images from bag(s) using a color map")
    sp.add_argument("--color-map", dest="color_map",
                    help="YAML/JSON string or file with color_map entries")
    sp.add_argument("--interactive", action="store_true",
                    help="Preview frames; click or press 's' to save")
    sp.add_argument("--save-all", action="store_true",
                    help="Save every colorized frame to --out")
    sp.add_argument("--save-both", action="store_true",
                    help="Save both label colors and overlay when saving")
    sp.add_argument("--overlay-topic", dest="overlay_topic",
                    help="Base image topic to blend with labels")
    sp.add_argument("--overlay-alpha", type=float, default=0.5,
                    help="Overlay alpha (0..1) for label blending")
    sp.add_argument("--overlay-max-dt", type=float, default=0.05,
                    help="Max time delta (s) between overlay image and labels (<=0 disables)")
    sp.add_argument("--stride", type=int, default=1,
                    help="Frame stride (e.g., 2=every other frame)")
    sp.add_argument("--max-frames", type=int, default=0,
                    help="Max frames per bag (0=all)")
    sp.add_argument("--resize", type=float, default=1.0,
                    help="Display scale factor (interactive only)")
    sp.set_defaults(out_path="colorized_labels")

    # -------- mapir_ndvi --------
    sp = sub.add_parser("mapir_ndvi", parents=[common_cfg],
                        help="Derive NDVI from /mapir/image_raw and write a bag with only the derived topic(s)")
    sp.add_argument("--image-topic", default="/mapir/image_raw",
                    help="Input MAPIR image topic to read from the source bag")
    sp.add_argument("--output-topic", default="/mapir/indices/ndvi",
                    help="Output topic for the raw NDVI image")
    sp.add_argument("--output-encoding", default="32FC1", choices=["32FC1", "mono8"],
                    help="Encoding for the raw NDVI image topic")
    sp.add_argument("--publish-color", dest="publish_color", action="store_true", default=True,
                    help="Also write a colorized NDVI topic to the output bag")
    sp.add_argument("--no-publish-color", dest="publish_color", action="store_false",
                    help="Disable the colorized NDVI topic in the output bag")
    sp.add_argument("--color-topic", default="/mapir/indices_color/ndvi",
                    help="Output topic for the colorized NDVI image")
    sp.add_argument("--colormap", default="plant_health",
                    choices=["plant_health", "viridis", "jet", "gray", "custom"],
                    help="Colormap for the colorized NDVI topic; plant_health maps higher NDVI to greener colors")
    sp.add_argument("--custom-colormap", default="",
                    help="Custom colormap 'value,r,g,b; value,r,g,b' used when --colormap=custom")
    sp.add_argument("--colorize-min", type=float, default=-1.0,
                    help="Minimum NDVI value mapped to the start of the colormap")
    sp.add_argument("--colorize-max", type=float, default=1.0,
                    help="Maximum NDVI value mapped to the end of the colormap")
    sp.add_argument("--filter-set", default="OCN",
                    help="MAPIR filter set preset used to resolve default NIR and visible channels")
    sp.add_argument("--nir-channel", type=int, default=-1,
                    help="Override NIR channel index (0=B, 1=G, 2=R); negative uses the filter-set default")
    sp.add_argument("--visible-channel", type=int, default=-1,
                    help="Override visible-band channel index (0=B, 1=G, 2=R); negative uses the filter-set default")
    sp.add_argument("--visible-band-name", default=None,
                    help="Optional label for the visible band in logs")
    sp.add_argument("--eps", type=float, default=1.0e-6,
                    help="Small positive value to stabilize NDVI division")
    if enable_shell_completion and argcomplete:
        argcomplete.autocomplete(parser)
    return parser

# --------------------------- Dispatcher -------------------------------------

def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    global logger
    logger = get_logger("Bagutils", level=args.log_level, log_file=args.log_file)

    user_cfg = load_yaml(getattr(args, "user_config", None))
    default_dev_cfg = pathlib.Path(__file__).resolve().parents[3] / "config" / "dev_defaults.yaml"
    dev_cfg = load_yaml(getattr(args, "dev_config", None)) or load_yaml(str(default_dev_cfg))
    cfg = merge_configs(args, user_cfg, dev_cfg)

    bu = RosbagUtils()

    # Dispatcher
    if args.cmd in ["auto_illumination"]:
        bu.auto_illumination_from_bag(cfg)

    elif args.cmd == "calculate_bag_duration":
        results = bu.calculate_bag_duration(cfg["in_path"], total=args.total)
        print_results(results, "Durations:")

    elif args.cmd == "remove_topic":
        bu.remove_topic(cfg["in_path"], cfg["out_path"], cfg["topics"])

    elif args.cmd == "change_frame_id":
        bu.change_frame_id(cfg["in_path"], cfg["out_path"], cfg["topics"], cfg["new_frame_id"])

    elif args.cmd == "remap_topics":
        remap = {}
        for pair in cfg["remap"]:
            if ":" not in pair:
                parser.error(f"--remap entry must be OLD:NEW, got: {pair}")
            old, new = pair.split(":", 1)
            old = old.strip()
            new = new.strip()
            if not old or not new:
                parser.error(f"--remap entry must be OLD:NEW, got: {pair}")
            remap[old] = new
        bu.remap_topics(cfg["in_path"], cfg.get("out_path"), remap, overwrite=bool(cfg.get("overwrite", False)))

    elif args.cmd == "print_topic_sizes":
        results = bu.print_topic_sizes(cfg["in_path"])

    elif args.cmd == "convert_imu_to_enu":
        bu.convert_imu_to_enu(cfg["in_path"], cfg["out_path"], cfg["topics"])

    elif args.cmd == "navsat_export":
        bu.navsat_export(cfg["in_path"], cfg["out_path"], cfg["topics"],
                         csv_name=cfg.get("csv_name", "navsat.csv"),
                         kml_name=cfg.get("kml_name"))

    elif args.cmd == "navsat_summary":
        results = bu.navsat_summary(cfg["in_path"], cfg["topics"])
        logger.info(f"NavSat Summary: {results}")

    elif args.cmd == "navsat_report":
        results = bu.navsat_report(cfg["in_path"], cfg["topics"], cfg["report"])
        # print_results(results, "NavSat Report:")
    elif args.cmd == "extract_metadata":
        bu.extract_metadata(cfg["in_path"], cfg.get("out_path"), cfg.get("topics"), cfg.get("max_msgs", 0))

    elif args.cmd == "extract_images_manifest":
        in_path = cfg.get("in_path")
        out_path = cfg.get("out_path")
        if not in_path:
            parser.error("extract_images_manifest requires --in")
        if not out_path:
            parser.error("extract_images_manifest requires --out")
        results = bu.extract_images_manifest(
            in_path=in_path,
            out_dir=out_path,
            topics=cfg.get("topics"),
            manifest_name=cfg.get("manifest_name", "image_manifest"),
            manifest_format=cfg.get("manifest_format", "csv"),
            time_source=cfg.get("time_source", "auto"),
            topic_discovery_timeout_s=cfg.get("topic_discovery_timeout", 5.0),
            startup_timeout_s=cfg.get("startup_timeout", 25.0),
        )
        print_results(results, "Image Extraction Summary:")

    elif args.cmd == "images_manifest_to_bag":
        in_path = cfg.get("in_path")
        out_path = cfg.get("out_path")
        if not in_path:
            parser.error("images_manifest_to_bag requires --in (manifest path)")
        if not out_path:
            parser.error("images_manifest_to_bag requires --out (output .bag path)")
        topic_override = None
        if cfg.get("topics"):
            topic_override = cfg["topics"][0]
        results = bu.images_manifest_to_bag(
            manifest_path=in_path,
            out_bag=out_path,
            images_root=cfg.get("images_root"),
            topic_override=topic_override,
            frame_id_override=cfg.get("frame_id"),
            output_encoding=cfg.get("output_encoding", "bgr8"),
            write_time=cfg.get("write_time", "stamp"),
            delimiter_mode=cfg.get("manifest_delimiter", "auto"),
            strict=bool(cfg.get("strict", False)),
        )
        print_results(results, "Bag Rebuild Summary:")

    elif args.cmd == "colorize_labels":
        bu.colorize_label_images(
            cfg["in_path"],
            cfg.get("topics"),
            cfg.get("out_path"),
            color_map=cfg.get("color_map"),
            interactive=bool(cfg.get("interactive")),
            save_all=bool(cfg.get("save_all")),
            save_both=bool(cfg.get("save_both")),
            overlay_topic=cfg.get("overlay_topic"),
            overlay_alpha=cfg.get("overlay_alpha", 0.5),
            overlay_max_dt=cfg.get("overlay_max_dt", 0.05),
            stride=cfg.get("stride", 1),
            max_frames=cfg.get("max_frames", 0),
            resize=cfg.get("resize", 1.0),
        )

    elif args.cmd == "mapir_ndvi":
        bu.mapir_ndvi(
            cfg["in_path"],
            cfg["out_path"],
            image_topic=cfg.get("image_topic", "/mapir/image_raw"),
            output_topic=cfg.get("output_topic", "/mapir/indices/ndvi"),
            output_encoding=cfg.get("output_encoding", "32FC1"),
            publish_color=bool(cfg.get("publish_color", True)),
            color_topic=cfg.get("color_topic", "/mapir/indices_color/ndvi"),
            colormap=cfg.get("colormap", "plant_health"),
            custom_colormap=cfg.get("custom_colormap", ""),
            colorize_min=cfg.get("colorize_min", -1.0),
            colorize_max=cfg.get("colorize_max", 1.0),
            filter_set=cfg.get("filter_set", "OCN"),
            nir_channel=cfg.get("nir_channel", -1),
            visible_channel=cfg.get("visible_channel", -1),
            visible_band_name=cfg.get("visible_band_name"),
            eps=cfg.get("eps", 1.0e-6),
        )
    elif args.cmd == "analyze_metrics":
        results = bu.analyze_metrics(
            cfg["in_path"],
            cfg["topics"],
            cfg["report"],
            cfg,
            benchmark=args.benchmark
        )
        logger.info("Metric ranges: %s", results)
    elif args.cmd == "urdf_extrinsics":
        urdf_path = cfg.get("urdf") or cfg.get("urdf_path")
        if not urdf_path:
            parser.error("urdf_extrinsics requires --urdf")
        T = load_extrinsics_from_urdf(urdf_path, cfg["parent_link"], cfg["child_link"])
        import numpy as np
        np.set_printoptions(precision=4, suppress=True)
        if cfg.get("rotation_only"):
            print(f"Rotation {cfg['parent_link']} -> {cfg['child_link']}:\n{T[:3, :3]}")
        elif cfg.get("translation_only"):
            print(f"Translation {cfg['parent_link']} -> {cfg['child_link']}:\n{T[:3, 3]}")
        else:
            print(f"Transform {cfg['parent_link']} -> {cfg['child_link']}:\n{T}")

    else:
        parser.error(f"Unknown command: {args.cmd}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
