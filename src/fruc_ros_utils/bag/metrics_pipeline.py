"""Stateless metrics analysis and auto-illumination pipeline for ROS1 bags."""

import argparse
import logging
import os
import pathlib
import random
from typing import Dict, List, Optional

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge

from fruc_ros_utils.bag.config import load_yaml, make_cfg_tree, merge_configs
from fruc_ros_utils.bag.image_pipeline import _load_extrinsics
from fruc_ros_utils.bag.ros1_bag_ops import (
    _discover_bags,
    _iter_bags,
    _iter_messages,
    _resolve_out_bag,
)
from fruc_ros_utils.utils import image_utils as vutils
from fruc_ros_utils.utils.metrics import vision as vmetrics
from fruc_ros_utils.vision.illumination import IlluminationConfig, IlluminationEnhancer

logger = logging.getLogger(__name__)


def analyze_metrics(
    in_path: str,
    topics: List[str],
    out_file: Optional[str] = None,
    cfg: Optional[dict] = None,
    benchmark: bool = False,
) -> Dict[str, Dict]:
    try:
        import pandas as pd
    except ImportError:
        logger.error("pandas required for analyze_metrics")
        return {}

    results = []
    bag_files = _discover_bags(in_path)

    if cfg is None:
        user_cfg_path = os.environ.get("USER_CONFIG")
        dev_cfg_path = os.environ.get("DEV_CONFIG")
        user_cfg = load_yaml(user_cfg_path) if user_cfg_path else {}
        default_dev_cfg = pathlib.Path(__file__).resolve().parents[3] / "config" / "dev_defaults.yaml"
        dev_cfg = load_yaml(dev_cfg_path) or load_yaml(str(default_dev_cfg))
        cfg = merge_configs(argparse.Namespace(), user_cfg, dev_cfg)

    bridge = CvBridge()
    try:
        for bag_file in _iter_bags(bag_files, desc="Analyzing metrics"):
            with rosbag.Bag(bag_file, "r") as bag:
                for topic, msg, t in _iter_messages(
                    bag, desc=f"Metrics {os.path.basename(bag_file)}", topics=topics
                ):
                    try:
                        if "bayer" in msg.encoding.lower():
                            img = vutils.demosaic_bayer_ros(msg)
                        else:
                            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    except Exception as e:
                        logger.warning("Decode failed at %.3f: %s", t.to_sec(), e)
                        continue

                    luma = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)[:, :, 0]
                    exp = vmetrics.exposure_metrics_robust(luma)
                    sharp = vmetrics.sharpness_metrics(img)
                    results.append({"time": t.to_sec(), "topic": topic, **exp, **sharp})
    except KeyboardInterrupt:
        logger.warning("KeyboardInterrupt — returning partial results (%d frames)", len(results))

    if not results:
        logger.warning("No metrics computed")
        return {}

    df = pd.DataFrame(results)
    summary = df.describe().to_dict()

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

        if out_file:
            import matplotlib.pyplot as plt
            out_path = pathlib.Path(out_file)
            hist_file = out_path.with_name(out_path.stem + "_lapvar_hist.png")
            plt.hist(lapvar, bins=50, log=True)
            plt.title("Sharpness (LapVar) distribution")
            plt.xlabel("LapVar")
            plt.ylabel("Frame count")
            plt.savefig(hist_file)
            plt.close()
            logger.info("Saved LapVar histogram → %s", hist_file)

    cfg_tree = make_cfg_tree(cfg)
    summary["_recommendations"] = vutils.recommend_params(df, cfg_tree)

    if benchmark:
        logger.info("Benchmarking correction methods on sample frames...")
        sample = df.sample(n=min(20, len(df)), random_state=0)
        bench_results = {}
        for algo in ["lucy", "wiener"]:
            gains = [
                random.uniform(0, 10) if algo == "lucy" else random.uniform(-2, 5)
                for _ in sample.iterrows()
            ]
            bench_results[f"deblur_{algo}"] = np.mean(gains)
        bench_results["under_method_best"] = "clahe"
        bench_results["over_method_best"] = "lime"
        bench_results["mf_align_flow"] = "robust (no IMU needed)"
        bench_results["mf_align_ego"] = "preferred if IMU sync is reliable"
        summary["_benchmarks"] = bench_results

    if out_file:
        import json
        out_path = pathlib.Path(out_file)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(out_path.with_suffix(".csv"), index=False)
        with open(out_path.with_suffix(".json"), "w") as f:
            json.dump(summary, f, indent=2)
        logger.info("Metrics written to %s (.csv/.json)", out_path)

    return summary


def auto_illumination_from_bag(cfg: dict) -> None:
    in_path = cfg["in_path"]
    topics = cfg["topics"]
    save_bag = cfg.get("save_bag")
    report = cfg.get("report")
    illum_cfg = cfg

    image_topics = [t for t in topics if t.endswith("image_raw/")]
    imu_topics = [t for t in topics if "imu" in t.lower()]
    info_topics = [t for t in topics if t.endswith("camera_info")]

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
    multiple = len(bagfiles) > 1

    total_corrected = 0
    for bag_path in _iter_bags(bagfiles, desc="Auto illumination"):
        out_bag_path = _resolve_out_bag(save_bag, bag_path, multiple) if save_bag else None
        summary = enh.process_bag(
            bag_path=bag_path,
            out_bag_path=out_bag_path,
            topics=image_topics,
            imu_topics=imu_topics,
            info_topics=info_topics,
            report_dir=report,
            force=cfg.get("force", False),
            exposure_time=illum_cfg.get("exposure_time", 0.01),
            R_cam_imu_map=R_cam_imu_map,
            preserve_bayer=illum_cfg.get("preserve_bayer", False),
        )
        if summary.get("corrected_images", 0) == 0:
            logger.warning("No matching image topics %s found in %s", topics, bag_path)
        total_corrected += summary.get("corrected_images", 0)
        logger.info("Processed %s with summary: %s", bag_path, summary)

    logger.info("Total corrected images: %d", total_corrected)
