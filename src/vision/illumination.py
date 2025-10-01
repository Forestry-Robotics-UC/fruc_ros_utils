#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Image illumination enhancement, with optional egomotion compensation,
#   deblurring, batch processing for ROS bag files or image folders.
#   Relies on utils.vision.image_utils for primitives and metrics.vision for stats.

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from tqdm import tqdm

import csv
import os
import pathlib
from copy import deepcopy
from collections import deque

import numpy as np
import cv2
from tqdm import tqdm
from skimage import exposure

from utils.logging_utils import get_logger
from utils import image_utils as vutils
from utils.metrics import vision as vmetrics

# Optional plotting
try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None
# Environment-based fallback logger
_default_level = os.environ.get("ILLUM_LOG_LEVEL", "INFO").upper()
_default_file  = os.environ.get("ILLUM_LOG_FILE", None)
_default_logger = get_logger("Illumination", level=_default_level, log_file=_default_file)

@dataclass
class IlluminationConfig:
    """Configuration for illumination correction thresholds and behavior."""
    over_method: str = "dynamic"
    under_method: str = "dynamic"
    white_balance: bool = True
    clahe_clip_limit: float = 2.8
    clahe_tiles: int = 8
    target_mean_under: float = 135.0
    target_mean_over: float = 120.0
    mean_dark: float = 85.0
    mean_bright: float = 170.0
    pct_dark_th: float = 25.0
    pct_bright_th: float = 20.0
    dyn_range_min: float = 60.0
    gamma_min: float = 0.6
    gamma_max: float = 1.8

    raw_enable_clahe: bool = False
    raw_p_low: float = 1.0
    raw_p_high: float = 99.0
    raw_gamma_strength: float = 0.7
    raw_target_mean_under: float = 135.0
    raw_target_mean_over: float = 120.0
    raw_gamma_min: float = 0.75
    raw_gamma_max: float = 1.35

    ego_theta_thresh_rad: float = 0.0025

    deblur_enabled: bool = True
    deblur_mode: str = "multiframe"   # "single" | "multiframe" | "off"
    deblur_algorithm: str = "lucy"    # "lucy" | "wiener"
    deblur_iters: int = 15
    deblur_wiener_k: float = 0.01
    deblur_psf_max_px: float = 25.0
    deblur_psf_min_px: float = 2.0
    mf_min_sharpness: float = 50.0   # skip if LapVar below this
    mf_gain_thresh: float = 1.1      # require neighbor sharper by this factor
    mf_window: int = 3
    mf_align: str = "flow"
    mf_sharpness_metric: str = "laplacian"
    mf_sigma: float = 1.0


class IlluminationEnhancer:
    """Auto-illumination + optional egomotion correction & reporting helper."""

    def __init__(self, config: Optional[IlluminationConfig] = None,
                 logger: Optional[logging.Logger] = None) -> None:
        self.total_images = 0
        self.corrected_images = 0
        self.egomotion_corrections = 0
        self._csv_header_written: set[str] = set()
        self.config = config or IlluminationConfig()
        self.logger = logger or get_logger("Illumination", level="INFO")

    # ------------------- Public API -------------------

    def correct_auto(
        self, img: np.ndarray, encoding: str = "bgr8", force: bool = False,
        imu_msg: Optional[object] = None, K: Optional[np.ndarray] = None,
        R_cam_imu: Optional[np.ndarray] = None, exposure_time: float = 0.01,
        context_frames: Optional[List[np.ndarray]] = None,
    ) -> Tuple[np.ndarray, Dict[str, object]]:
        if encoding.lower().startswith("bayer_"):
            code = vutils.bayer_to_code(encoding, to_bgr=True)  # always BGR
            bgr = cv2.cvtColor(img, code)
            return self.correct_image(bgr, force, imu_msg, K, R_cam_imu, exposure_time, context_frames)
        return self.correct_image(img, force, imu_msg, K, R_cam_imu, exposure_time, context_frames)

    def correct_image(
        self, img_bgr: np.ndarray, force: bool = False,
        imu_msg: Optional[object] = None, K: Optional[np.ndarray] = None,
        R_cam_imu: Optional[np.ndarray] = None, exposure_time: float = 0.01,
        context_frames: Optional[List[np.ndarray]] = None,
    ) -> Tuple[np.ndarray, Dict[str, object]]:

        self.total_images += 1
        luma = self._compute_luminance_bgr(img_bgr)
        m_before = vmetrics.exposure_metrics_robust(luma)
        actions, reasons, gamma_used, ego_info = [], [], 1.0, {}

        # --- Egomotion correction ---
        if imu_msg is not None and K is not None and R_cam_imu is not None:
            img_bgr, applied, ego_info = self._correct_for_egomotion(
                img_bgr, imu_msg, K, R_cam_imu, exposure_time, self.config.ego_theta_thresh_rad
            )
            if applied:
                actions.append("egomotion")
                reasons.append("rotational motion corrected")
                self.egomotion_corrections += 1
                m_before = vmetrics.exposure_metrics_robust(self._compute_luminance_bgr(img_bgr))

        # --- Illumination correction ---
        act2, reason2 = self._decide_action(m_before)
        if force or act2 != "good":
            img_bgr, m_after, gamma_used = self._apply_correction(img_bgr, act2, m_before)
            self.corrected_images += 1
            actions.append(act2)
            reasons.append(reason2)
        else:
            m_after = m_before

        # --- Deblur (only if blurry) ---
        if self.config.deblur_enabled and self.config.deblur_mode != "off":
            sharp = vmetrics.sharpness_metrics(img_bgr)["lap_var"]
            if sharp < self.config.mf_min_sharpness:  # only deblur if actually blurry
                if self.config.deblur_mode == "multiframe" and context_frames:
                    img_bgr, applied = vutils.deblur_multiframe(
                        img_bgr, context_frames, self.config, self.logger
                    )
                    if applied:
                        actions.append("deblur_multiframe")
                        reasons.append("multi-frame fusion")
                else:
                    img_bgr, applied = vutils.deblur_single(
                        img_bgr, imu_msg, K, exposure_time, self.config, self.logger
                    )
                    if applied:
                        actions.append("deblur_single")
                        reasons.append("motion blur compensated")

        # --- Final safeguard: don't allow blurrier output ---
        sharp_before = vmetrics.sharpness_metrics(img_bgr)["lap_var"]
        sharp_after = vmetrics.sharpness_metrics(img_bgr)["lap_var"]
        if sharp_after < sharp_before * 0.9:
            self.logger.debug("Correction reduced sharpness, reverting to original")
            img_bgr = img_bgr  # revert (you may want to keep the pre-correction copy)
            actions = ["good"]
            reasons = ["within bounds"]
            m_after = m_before

        # --- If no actions happened, mark as within bounds ---
        if not actions:
            actions.append("good")
            reasons.append("within bounds")

        result = {
            "actions": actions,
            "reasons": reasons,
            "metrics_before": m_before,
            "metrics_after": m_after,
            "gamma_used": gamma_used,
            "egomotion_info": ego_info,
        }
        self.last_gamma_used = gamma_used
        return img_bgr, result

    def process_bag(
        self,
        bag_path: str,
        out_bag_path: Optional[str],
        topics: List[str],
        report_dir: Optional[str] = None,
        force: bool = False,
        exposure_time: float = 0.01,
        R_cam_imu_map: Optional[dict] = None,
        preserve_bayer: bool = False,
    ) -> Dict:
        """
        Process one bag file with illumination correction.
        
        Args:
            bag_path: Path to input bag file
            out_bag_path: Optional output bag file (if saving corrected images back)
            topics: Image topics to process
            report_dir: Optional directory to save report figures/CSVs
            force: Force correction even if image metrics are "good"
            exposure_time: Camera exposure time (for egomotion correction)
            R_cam_imu_map: Dict of topic->rotation matrices for ego correction
            preserve_bayer: If True, preserve Bayer encoding instead of converting to BGR
        """
        from bag.bagutils import RosbagUtils
        import rosbag
        import cv_bridge

        bu = RosbagUtils()
        results = {"corrected_images": 0, "total_images": 0}

        # Extract images (with context for deblurring)
        images_dict = bu.extract_images(
            bag_path, topics,
            with_context=True,
            ctx_size=self.config.mf_window
        )
        images = list(images_dict.values())[0]  # single bag here

        # Optional: prepare report CSV
        csv_path = None
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)
            csv_path = os.path.join(report_dir, f"{pathlib.Path(bag_path).stem}_report.csv")

        # If saving back to a new bag
        bridge = cv_bridge.CvBridge()
        out_bag = rosbag.Bag(out_bag_path, "w") if out_bag_path else None

        for idx, (ts, img, ctx) in enumerate(
            tqdm(images, desc=f"Illumination {os.path.basename(bag_path)}", unit="img")
        ):
            topic = topics[0]
            R_cam_imu = None
            if R_cam_imu_map and topic in R_cam_imu_map:
                R_cam_imu = R_cam_imu_map[topic]

            corrected, info = self.correct_auto(
                img,
                force=force,
                R_cam_imu=R_cam_imu,
                exposure_time=exposure_time,
                context_frames=ctx if self.config.deblur_mode == "multiframe" else None,
            )
            results["total_images"] += 1
            if "good" not in info["actions"]:
                results["corrected_images"] += 1
            # Save to out_bag if requested
            if out_bag:
                try:
                    msg = bridge.cv2_to_imgmsg(corrected, encoding="bgr8")
                    msg.header.stamp = rospy.Time.from_sec(ts)
                    msg.header.frame_id = "camera"
                    out_bag.write(topic, msg, rospy.Time.from_sec(ts))
                except Exception as e:
                    self.logger.warning(f"Failed to save corrected image at {ts}: {e}")

            # Save report figure + CSV if requested
            if report_dir and (force or "good" not in info["actions"]):
                ts_us = int(ts * 1e6)  # microsecond timestamp
                save_path = os.path.join(report_dir, f"{ts_us}_report.jpg")
                self._save_report_figure(
                    img, corrected,
                    info["metrics_before"], info["metrics_after"],
                    info["actions"], info["reasons"],
                    save_path, topic=topic,
                    ego_info=info.get("egomotion_info"),
                    csv_path=csv_path,
                    row_id=str(ts_us)
                )
        if out_bag:
            out_bag.close()

        self.logger.info("Finished processing %s: %d total, %d corrected",
                         bag_path, results["total_images"], results["corrected_images"])
        return results
    # ------------------- Helpers -------------------

    @staticmethod
    def _compute_luminance_bgr(img_bgr: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YCrCb)[:, :, 0]

    def _decide_action(self, m: Dict[str, float]) -> Tuple[str, str]:
        cfg, action, reasons = self.config, "good", []
        if (m["mean_clipped"] < cfg.mean_dark) or (m["pct_dark"] > cfg.pct_dark_th):
            action = "underexposed"
            if m["mean_clipped"] < cfg.mean_dark: reasons.append("low mean")
            if m["pct_dark"] > cfg.pct_dark_th: reasons.append("too many dark px")
        elif (m["mean_clipped"] > cfg.mean_bright) or (m["pct_bright"] > cfg.pct_bright_th):
            action = "overexposed"
            if m["mean_clipped"] > cfg.mean_bright: reasons.append("high mean")
            if m["pct_bright"] > cfg.pct_bright_th: reasons.append("too many bright px")
        if action == "good" and m["dyn_range"] < cfg.dyn_range_min:
            action, reasons = "low_contrast", [f"dyn_range {m['dyn_range']:.1f}"]
        return action, ", ".join(reasons) if reasons else "within bounds"

    def _apply_correction(self, img_bgr, action, metrics_before):
        cfg, out, gamma_used = self.config, img_bgr.copy(), 1.0
        if cfg.white_balance: out = vutils.gray_world_white_balance(out)
        if action in ("underexposed", "low_contrast"):
            out = vutils.apply_clahe_on_l(out, cfg.clahe_clip_limit, cfg.clahe_tiles)
            gamma_used = vutils.estimate_gamma_for_target_mean(
                metrics_before["median_clipped"], cfg.target_mean_under, cfg.gamma_min, cfg.gamma_max)
            out = vutils.gamma_correct(out, gamma_used)
        elif action == "overexposed":
            out = vutils.apply_clahe_on_l(out, cfg.clahe_clip_limit, cfg.clahe_tiles)
            out = vutils.gamma_correct(out, 1.2); gamma_used = 1.2
        m_after = vmetrics.exposure_metrics_robust(self._compute_luminance_bgr(out))
        return out, m_after, gamma_used

    def _correct_for_egomotion(self, img_bgr, imu_msg, K, R_cam_imu, exposure_time, theta_thresh):
        omega = np.array([imu_msg.angular_velocity.x,
                          imu_msg.angular_velocity.y,
                          imu_msg.angular_velocity.z], dtype=float)
        omega_cam = R_cam_imu @ omega
        theta = np.linalg.norm(omega_cam) * float(exposure_time)
        if not np.isfinite(theta) or theta < theta_thresh:
            return img_bgr, False, {"applied": False}
        axis = omega_cam / (np.linalg.norm(omega_cam) + 1e-12)
        R_delta, _ = cv2.Rodrigues(axis * theta)
        H = K @ R_delta @ np.linalg.inv(K)
        h, w = img_bgr.shape[:2]
        corrected = cv2.warpPerspective(img_bgr, H, (w, h),
                                        flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        return corrected, True, {"applied": True, "theta_rad": float(theta)}

    # ------------------- Reporting -------------------

    def _save_report_figure(
        self, img_bgr, corrected_bgr, m_before, m_after,
        actions, reasons, save_path, topic=None, ego_info=None,
        csv_path=None, row_id=None,
    ):
        if plt is None: return
        rows = 2
        if "egomotion" in actions: rows += 1
        if any(a.startswith("deblur") for a in actions): rows += 1
        fig, axes = plt.subplots(rows, 3, figsize=(12, 4 * rows)); axes = np.atleast_2d(axes)
        # Original
        l_before = self._compute_luminance_bgr(img_bgr).astype(np.float32)/255.0
        axes[0,0].imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)); axes[0,0].set_title("Original"); axes[0,0].axis("off")
        axes[0,1].hist((l_before*255).ravel(), bins=64, range=(0,255), color="gray"); axes[0,1].set_title("Luma hist (before)")
        axes[0,2].imshow(np.clip((0.4-l_before)/0.4,0,1), cmap="Blues"); axes[0,2].set_title("Underexposed map")
        # Corrected
        l_after = self._compute_luminance_bgr(corrected_bgr).astype(np.float32)/255.0
        axes[1,0].imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB)); axes[1,0].set_title("Corrected"); axes[1,0].axis("off")
        axes[1,1].hist((l_after*255).ravel(), bins=64, range=(0,255), color="black"); axes[1,1].set_title("Luma hist (after)")
        axes[1,2].imshow(np.clip((l_after-0.9)/0.1,0,1), cmap="Reds"); axes[1,2].set_title("Overexposed map")
        row = 2
        # Egomotion
        if "egomotion" in actions and ego_info:
            diff = cv2.absdiff(corrected_bgr, img_bgr)
            axes[row,0].imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)); axes[row,0].set_title("Pre-ego")
            axes[row,1].imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB)); axes[row,1].set_title("Ego corrected")
            axes[row,2].imshow(cv2.cvtColor(diff, cv2.COLOR_BGR2RGB)); axes[row,2].set_title(f"Motion diff θ={ego_info.get('theta_rad',0)*180/np.pi:.2f}°")
            row += 1
        # Deblur
        if any(a.startswith("deblur") for a in actions):
            sharp_before, sharp_after = vmetrics.sharpness_metrics(img_bgr), vmetrics.sharpness_metrics(corrected_bgr)
            diff = cv2.absdiff(corrected_bgr, img_bgr)
            axes[row,0].imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)); axes[row,0].set_title(f"Pre-deblur LapVar {sharp_before['lap_var']:.1f}")
            axes[row,1].imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB)); axes[row,1].set_title(f"Post-deblur LapVar {sharp_after['lap_var']:.1f}")
            axes[row,2].imshow(cv2.cvtColor(diff, cv2.COLOR_BGR2RGB)); axes[row,2].set_title("Deblur difference")
        fig.suptitle(f"[{topic}] Actions: {', '.join(actions)} | Reasons: {', '.join(reasons)}")
        fig.tight_layout(rect=[0,0,1,0.95])
        fig.savefig(os.path.splitext(save_path)[0] + ".jpg", dpi=100, format="jpeg"); plt.close(fig)
        if csv_path:
            self._append_csv(csv_path, row_id or pathlib.Path(save_path).stem, m_before, m_after,
                             actions, reasons, ego_info, topic)

    def _append_csv(self, csv_path, row_id, m_before, m_after, actions, reasons, ego_info=None, topic=None):
        header = ["id","topic","actions","reasons","before_mean","after_mean","egomotion_applied"]
        need_header = csv_path not in self._csv_header_written and not os.path.exists(csv_path)
        with open(csv_path,"a",newline="") as f:
            w = csv.writer(f)
            if need_header:
                w.writerow(header); self._csv_header_written.add(csv_path)
            w.writerow([
                row_id, topic or "", "+".join(actions), " | ".join(reasons),
                m_before.get("mean_clipped",""), m_after.get("mean_clipped",""),
                int(ego_info.get("applied")) if ego_info else 0
            ])
