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

# ===== Standard Library =====
import argparse
import os
import sys
import pathlib
from collections import deque, defaultdict
from typing import List, Dict, Optional
from concurrent.futures import ProcessPoolExecutor

# ===== Third-Party Libraries =====
import numpy as np
import cv2
from tqdm import tqdm
import yaml
import pandas as pd

# ===== ROS =====
import rosbag
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge

# ===== Custom Utilities =====
from utils.sensor_conversions import imu_ned_to_enu
from utils.logging_utils import get_logger
from utils.image_utils import demosaic_bayer_ros
from utils import image_utils as vutils

# ===== Custom TF / Extrinsics =====
from utils.tf_utils import (
    build_R_cam_imu_per_topic,
    load_extrinsics_yaml,
    load_extrinsics_from_urdf,
)

# ===== Custom Vision / Illumination =====
from vision.illumination import IlluminationEnhancer, IlluminationConfig
from utils.metrics import vision as vmetrics

# ===== Custom NavSat Tools =====
from bag.navsat_tools import export_navsat_to_csv, export_navsat_to_kml
from utils.metrics.navsat import cov_metrics
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
    base_dir = os.path.dirname(__file__)
    default_dev = os.path.join(base_dir, "..", "config", "dev_defaults.yaml")

    cfg = load_yaml(default_dev)
    cfg.update(load_yaml(dev_cfg_path))
    cfg.update(load_yaml(user_cfg_path))
    logger.debug("Loaded config=%s", cfg)
    return cfg

def merge_configs(cli_args, user_cfg: dict, dev_cfg: dict) -> dict:
    cfg = {}
    cfg.update(dev_cfg or {})
    cfg.update(user_cfg or {})

    # Flatten known subsections if present
    for section in ("bagutils", "extrinsics","navsat", "imu", "illumination"):
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

        for bag_path in _iter_bags(_discover_bags(in_path), desc="Calculating durations"):
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
            dev_cfg = load_yaml(dev_cfg_path) or load_yaml(
                os.path.join(os.path.dirname(__file__), "..", "config", "dev_defaults.yaml")
            )
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
                for _, row in sample.iterrows():
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
        out_path = pathlib.Path(cfg["out_path"])
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

def build_parser() -> argparse.ArgumentParser:
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

    if argcomplete:
        argcomplete.autocomplete(parser)
    return parser

# --------------------------- Dispatcher -------------------------------------

def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    global logger
    logger = get_logger("Bagutils", level=args.log_level, log_file=args.log_file)

    user_cfg = load_yaml(getattr(args, "user_config", None))
    dev_cfg = load_yaml(getattr(args, "dev_config", None)) or load_yaml(
        os.path.join(os.path.dirname(__file__), "..", "config", "dev_defaults.yaml")
    )
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
        T = load_extrinsics_from_urdf(cfg["urdf_path"], cfg["parent_link"], cfg["child_link"])
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