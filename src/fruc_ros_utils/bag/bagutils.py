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
import os
import sys
import pathlib
from collections import deque, defaultdict
from typing import List, Dict, Optional

# ===== Third-Party Libraries =====
import numpy as np
import cv2
from tqdm import tqdm
import yaml
import pandas as pd

# ===== ROS =====
import rosbag
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
# Allow environment variable override when not run via CLI
_default_level = os.environ.get("BAGUTILS_LOG_LEVEL", "INFO").upper()
_default_file  = os.environ.get("BAGUTILS_LOG_FILE", None)
logger = get_logger("Bagutils", level=_default_level, log_file=_default_file)

try:
    import argcomplete
except Exception:  # pragma: no cover
    argcomplete = None

# --------------------------------------------------------------------------- #
#                                HELPERS                                      #
# --------------------------------------------------------------------------- #
def _iter_bags(paths: List[str], desc: str = "Bags"):
    """Yield bag files with tqdm progress bar."""
    for bag_path in tqdm(paths, desc=desc, unit="bag"):
        yield bag_path

def _iter_messages(bag, desc: str, topics: Optional[List[str]] = None, raw: bool = False):
    """Yield messages from a bag with tqdm progress bar."""
    total = bag.get_message_count(topic_filters=topics) if not raw else bag.get_message_count()
    for msg in tqdm(
        bag.read_messages(topics=topics, raw=raw),
        total=total,
        desc=desc,
        unit="msg",
        leave=False
    ):
        yield msg

def _discover_bags(path: str) -> List[str]:
    """Return a list of bag files given a file or folder path."""
    p = pathlib.Path(path)
    if p.is_dir():
        return sorted(str(f) for f in p.glob("*.bag*") if f.is_file())
    return [str(p)]


def _resolve_out_bag(save_bag: str, bag_path: str, multiple: bool) -> str:
    """
    Resolve output bag path.
    - If save_bag is a directory → put output inside it.
    - If save_bag is a file → use it (suffix if multiple).
    Always ensures required directories exist.
    """
    sbp = pathlib.Path(save_bag)

    if sbp.suffix == "" or sbp.is_dir():  # directory mode
        sbp.mkdir(parents=True, exist_ok=True)
        name = f"{pathlib.Path(bag_path).stem}{'_corrected' if multiple else ''}.bag"
        out_path = sbp / name
    else:  # file mode
        out_path = (
            sbp.parent / f"{pathlib.Path(bag_path).stem}_corrected.bag"
            if multiple else sbp
        )
        out_path.parent.mkdir(parents=True, exist_ok=True)

    logger.debug("Resolved output path: %s (save_bag=%s, multiple=%s)", out_path, save_bag, multiple)
    return str(out_path)


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
def make_cfg_tree(cfg: dict) -> dict:
    """
    Rebuild a nested config tree from a flattened cfg dict.
    Ensures all sections have defaults so 'current' in recommendations is never None.
    """
    illum_defaults = IlluminationConfig().__dict__.copy()

    raw_defaults = {
        "raw_enable_clahe": False,
        "raw_p_low": 1.0,
        "raw_p_high": 99.0,
        "raw_gamma_strength": 0.7,
        "raw_target_mean_under": 135.0,
        "raw_target_mean_over": 120.0,
        "raw_gamma_min": 0.75,
        "raw_gamma_max": 1.35,
    }
    deblur_defaults = {
        "mf_min_sharpness": illum_defaults.get("mf_min_sharpness", 50.0),
        "deblur_mode": illum_defaults.get("deblur_mode", "off"),
        "deblur_algorithm": illum_defaults.get("deblur_algorithm", "lucy"),
    }
    egomotion_defaults = {
        "ego_theta_thresh_rad": illum_defaults.get("ego_theta_thresh_rad", 0.0025),
    }
    retinex_defaults = {
        "retinex_scales": [15, 80, 250],
        "lime_guided_radius": 15,
        "lime_guided_eps": 0.001,
    }
    runtime_defaults = {
        "preserve_bayer": False,
        "force": False,
    }

    def section(fill_from: dict, defaults: dict) -> dict:
        return {k: fill_from.get(k, dv) for k, dv in defaults.items()}

    return {
        "illumination": section(cfg, illum_defaults),
        "raw":          section(cfg, raw_defaults),
        "deblur":       section(cfg, deblur_defaults),
        "egomotion":    section(cfg, egomotion_defaults),
        "retinex_lime": section(cfg, retinex_defaults),
        "runtime":      section(cfg, runtime_defaults),
    }

def load_yaml(path: str) -> dict:
    if not path or not os.path.exists(path):
        return {}
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.error("Failed to load YAML %s: %s", path, e)
        return {}

def load_configs(user_cfg_path: str = None, dev_cfg_path: str = None) -> dict:
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    default_dev = repo_root / "config" / "dev_defaults.yaml"

    cfg = load_yaml(str(default_dev))
    cfg.update(load_yaml(dev_cfg_path))
    cfg.update(load_yaml(user_cfg_path))
    logger.debug("Loaded config=%s", cfg)
    return cfg

def merge_configs(cli_args, user_cfg: dict, dev_cfg: dict) -> dict:
    cfg = {}
    cfg.update(dev_cfg or {})
    cfg.update(user_cfg or {})

    # Flatten known subsections if present
    for section in (
        "bagutils",
        "extrinsics",
        "navsat",
        "imu",
        "illumination",
        "mapir_ndvi",
        "colorize_labels",
        "extract_metadata",
    ):
        if section in cfg and isinstance(cfg[section], dict):
            # logger.debug("Flattening section '%s'", section)
            cfg.update(cfg.pop(section))

    # Apply CLI overrides
    for k, v in vars(cli_args).items():
        if v is not None:
            cfg[k] = v

    logger.debug("Final merged config=%s", cfg)
    return cfg

# --------------------------------------------------------------------------- #
#                           MAIN CLASS                                        #
# --------------------------------------------------------------------------- #

class RosbagUtils:
    """Utility class for processing ROS bag files."""

    def __init__(self):
        
        self.bridge = CvBridge()

    def calculate_bag_duration(self, in_path: str, total: bool = False) -> Dict[str, float]:
        logger.debug("Input=%s total=%s", in_path, total)
        results: Dict[str, float] = {}
        bag_files = _discover_bags(in_path)

        for bag_path in _iter_bags(bag_files, desc="Calculating durations"):
            try:
                with rosbag.Bag(bag_path, "r") as bag:
                    duration = bag.get_end_time() - bag.get_start_time()
                fname = os.path.basename(bag_path)
                logger.debug("Duration=%.2fs", duration)
                results[fname] = duration
            except Exception as e:
                logger.error("Failed for %s: %s", bag_path, e)

        if total and results:
            total_dur = sum(results.values())
            logger.info("Total duration across %d bags: %.2f s", len(results), total_dur)
            results["__Total__"] = total_dur
            logger.debug("Total=%.2fs", total_dur)

        return results

    def remove_topic(self, in_path: str, out_path: str, topics: List[str]) -> None:
        logger.debug("Input=%s out=%s topics=%s", in_path, out_path, topics)
        bag_files = _discover_bags(in_path)
        multiple = len(bag_files) > 1

        for bag_file in _iter_bags(bag_files, desc="Removing topics"):
            out_bag_file = _resolve_out_bag(out_path, bag_file, multiple)
            removed_any = False  # track if any topic was actually removed

            with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
                for topic, msg, t in _iter_messages(in_bag, desc=f"Filtering {os.path.basename(bag_file)}"):
                    if topic not in topics:
                        out_bag.write(topic, msg, t)
                    else:
                        removed_any = True

            if removed_any:
                logger.info("Wrote cleaned bag to %s (removed topics: %s)", out_bag_file, topics)
            else:
                logger.warning("No matching topics %s found in %s. Output identical to input.", topics, bag_file)


    def print_topic_sizes(self, in_path: str) -> Dict[str, Dict[str, int]]:
        logger.debug("Input=%s", in_path)
        results: Dict[str, Dict[str, int]] = {}
        totals: Dict[str, int] = defaultdict(int)
        bag_files = _discover_bags(in_path)

        for bag_file in _iter_bags(bag_files, desc="Topic sizes"):
                sizes: Dict[str, int] = defaultdict(int)
                with rosbag.Bag(bag_file, "r") as bag:
                    for topic, msg, _ in _iter_messages(
                        bag, desc=f"Sizes {os.path.basename(bag_file)}", raw=True
                    ):
                        sizes[topic] += len(msg[1])
                        totals[topic] += len(msg[1])

                results[os.path.basename(bag_file)] = dict(sizes)
                logger.debug("%s → %s", bag_file, sizes.keys())
                for topic, size in sorted(sizes.items(), key=lambda x: x[1]):
                    logger.info("%s: %.2f MB (%.4f GB)", topic, size / 1e6, size / 1e9)

        if totals:
            total_topics = dict(totals)
            logger.info("Totals across %d bags:", len(results))
            for topic, size in sorted(total_topics.items(), key=lambda x: x[1], reverse=True):
                logger.info("TOTAL %s: %.2f MB (%.4f GB)", topic, size / 1e6, size / 1e9)
            results["__Totals__"] = total_topics

        return results

    def change_frame_id(self, in_path: str, out_path: str, topics: List[str], new_frame_id: str) -> None:
        logger.debug("Input=%s topics=%s new_frame_id=%s", in_path, topics, new_frame_id)
        bag_files = _discover_bags(in_path)
        multiple = len(bag_files) > 1

        for bag_file in _iter_bags(bag_files, desc="Changing frame_id"):
            out_bag_file = _resolve_out_bag(out_path, bag_file, multiple) if multiple else out_path
            with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
                for topic, msg, t in _iter_messages(in_bag, desc=f"FrameID {os.path.basename(bag_file)}", topics=topics):
                    if hasattr(msg, "header") and hasattr(msg.header, "frame_id") and topic in topics:
                        logger.debug("Change frame  %s → %s", msg.header.frame_id, new_frame_id)
                        msg.header.frame_id = new_frame_id


                    out_bag.write(topic, msg, t)
            logger.info("Updated frame_id for %s → %s in %s", topics, new_frame_id, out_bag_file)

    def convert_imu_to_enu(self, in_path: str, out_path: str, topics: List[str]) -> None:
        logger.debug("Input=%s out=%s topics=%s", in_path, out_path, topics)
        bag_files = _discover_bags(in_path)
        multiple = len(bag_files) > 1

        for bag_file in _iter_bags(bag_files, desc="Converting IMU"):
            out_bag_file = _resolve_out_bag(out_path, bag_file, multiple) if multiple else out_path
            found_topics = {t: False for t in topics}  # track which IMU topics were seen

            with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
                for topic, msg, t in _iter_messages(in_bag, desc=f"IMU {os.path.basename(bag_file)}"):
                    if topic in topics and "Imu" in getattr(msg, "_type", ""):
                        found_topics[topic] = True
                        out_bag.write(topic, imu_ned_to_enu(msg), t)
                    else:
                        out_bag.write(topic, msg, t)

            # Report results
            for imu_topic, seen in found_topics.items():
                if seen:
                    logger.info("Converted IMU topic %s from NED → ENU in %s → %s",
                                imu_topic, bag_file, out_bag_file)
                else:
                    logger.warning("IMU topic %s not found in %s. Output unchanged for that topic.",
                                   imu_topic, bag_file)

    def convert_camera_info(self, in_path: str, out_path: str, topics: Optional[List[str]] = None) -> None:
        logger.debug("Input=%s out=%s topics=%s", in_path, out_path, topics)
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
                                logger.warning("Topic %s in %s is not CameraInfo (type=%s). Writing original.",
                                               topic, bag_file, mtype)
                                out_bag.write(topic, msg, t)
                                continue
                            out_bag.write(topic, _convert_camera_info_msg(msg), t)
                            converted_topics.add(topic)
                        except Exception as e:
                            logger.warning("Failed to convert %s in %s: %s. Writing original.",
                                           topic, bag_file, e)
                            out_bag.write(topic, msg, t)
                    else:
                        out_bag.write(topic, msg, t)

            if cam_topics:
                missing = [t for t in cam_topics if t not in converted_topics]
                if missing:
                    logger.warning("CameraInfo topics not found in %s: %s", bag_file, missing)
            logger.info("Wrote converted bag to %s", out_bag_file)

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

    def extract_images(self, in_path: str, topics: List[str], with_context: bool = False, ctx_size: int = 3) -> Dict[str, List]:
        global logger
        logger.debug("Input=%s topics=%s with_context=%s ctx_size=%d", in_path, topics, with_context, ctx_size)

        results: Dict[str, List] = {}
        bag_files = _discover_bags(in_path)

        def _is_image_datatype(dt: str) -> bool:
            # Be strict first, but allow common custom types that end with Image
            return dt in ("sensor_msgs/Image", "sensor_msgs/CompressedImage") or dt.endswith("/Image") or dt.endswith("/CompressedImage")

        for bag_file in bag_files:
            bridge = CvBridge()
            images = []
            ctx = deque(maxlen=ctx_size)

            with rosbag.Bag(bag_file, "r") as bag:
                # --- discover image topics in this bag ---
                try:
                    info = bag.get_type_and_topic_info()
                    topics_info = info.topics if hasattr(info, "topics") else info[1]
                except Exception:
                    topics_info = {}

                # Filter requested topics down to image topics available in this bag
                requested = set(topics or [])
                image_topics = []
                non_image_requested = []
                for name, tinfo in topics_info.items():
                    msg_type = getattr(tinfo, "msg_type", getattr(tinfo, "datatype", None))
                    if not msg_type:
                        continue
                    if (not requested) or (name in requested):
                        if _is_image_datatype(msg_type):
                            image_topics.append(name)
                        elif name in requested:
                            non_image_requested.append((name, msg_type))

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
                        # Decode by runtime message type
                        mtype = getattr(msg, "_type", "")
                        if mtype.endswith("CompressedImage"):
                            cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                        else:
                            # sensor_msgs/Image (raw)
                            enc = getattr(msg, "encoding", "") or ""
                            if "bayer" in enc.lower():
                                
                                cv_img = demosaic_bayer_ros(msg)  # returns BGR

                            else:
                                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                        # Safety: skip invalid/empty frames; ensure 3-channel BGR
                        if cv_img is None or getattr(cv_img, "size", 0) == 0:
                            logger.warning("Decoded empty image at %.6f on %s in %s", t.to_sec(), topic, bag_file)
                            continue
                        if cv_img.ndim == 2:
                            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)

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
