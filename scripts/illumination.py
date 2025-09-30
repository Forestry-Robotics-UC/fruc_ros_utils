#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
illumination.py — Auto-illumination enhancement with optional egomotion compensation

Features:
  • Assess exposure/contrast on images (robust stats)
  • Correct illumination only when needed (CLAHE + gamma, optional gray-world WB)
  • Optionally correct rotational egomotion using IMU + intrinsics + extrinsics
  • Generate per-image reports with clear layout (text outside image)
  • Batch & preview helpers
  • Summary CSV + graphs of changed images

Notes:
  • Egomotion correction here compensates *pure rotational* motion during exposure. Translation
    generally requires scene depth and is out-of-scope for this lightweight step.
  • Egomotion correction is applied only if the estimated rotation during exposure exceeds a
    small threshold (default ~0.57°), to avoid unnecessary warping.
"""

from __future__ import annotations
from skimage import exposure
from copy import deepcopy
import csv
import logging
import os
import pathlib
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from collections import deque

import cv2
import numpy as np
from tqdm import tqdm

# Optional plotting
try:  # pragma: no cover - optional
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - optional
    plt = None

# Optional custom logger
try:  # pragma: no cover - optional
    from custom_logging import setup_custom_logger
except Exception:  # pragma: no cover - optional
    setup_custom_logger = None


@dataclass
class IlluminationConfig:
    """Configuration for illumination correction thresholds and behavior."""
    # Exposure correction method selectors
    over_method: str = "dynamic"   # options: "dynamic", "reinhard", "drago", "mantiuk", "clahe", "msr", "lime"
    under_method: str = "dynamic"  # options: "dynamic", "clahe", "gamma", "msr", "lime"
    white_balance: bool = True
    clahe_clip_limit: float = 2.8
    clahe_tiles: int = 8

    # Targets for gamma correction
    target_mean_under: float = 135.0
    target_mean_over: float = 120.0

    # Decision thresholds (8-bit luminance)
    mean_dark: float = 85.0
    mean_bright: float = 170.0
    pct_dark_th: float = 25.0    # % of pixels ≤ 20
    pct_bright_th: float = 20.0  # % of pixels ≥ 235
    dyn_range_min: float = 60.0  # p99 − p1

    # Gamma clamps
    gamma_min: float = 0.6

    gamma_max: float = 1.8
    # RAW (Bayer) tonemap strategy (single-channel, pre-demosaic)
    raw_enable_clahe: bool = False        # default OFF: avoid local EQ on mosaics
    raw_p_low: float = 1.0                # percentile for low clip
    raw_p_high: float = 99.0              # percentile for high clip
    raw_gamma_strength: float = 0.7       # exponent to move toward target (mild)
    raw_target_mean_under: float = 135.0  # same targets as color path
    raw_target_mean_over: float = 120.0
    raw_gamma_min: float = 0.75
    raw_gamma_max: float = 1.35
    # Egomotion
    ego_theta_thresh_rad: float = 0.0025  # ~0.57°

    # --- Deblur options (classic) ---
    deblur_enabled: bool = True
    deblur_mode: str = "single"     # "single" | "multiframe" | "off"
    deblur_algorithm: str = "lucy"  # "lucy" | "wiener"
    deblur_iters: int = 15          # RL iterations (typ. 10–30)
    deblur_wiener_k: float = 0.01   # Wiener constant (noise-to-signal)
    deblur_psf_max_px: float = 25.0 # cap on PSF length in pixels
    deblur_psf_min_px: float = 2.0  # ignore trivial blur
    # for multiframe
    mf_window: int = 3              # odd numbers: 3,5 (center + neighbors)
    mf_align: str = "flow"          # "flow" | "ego"
    mf_sharpness_metric: str = "laplacian"  # "laplacian" | "sobel"
    mf_sigma: float = 1.0           # smoothing of weights


class IlluminationEnhancer:
    """Auto-illumination + optional egomotion correction & reporting helper."""

    def __init__(
        self,
        config: Optional[IlluminationConfig] = None,
        logger: Optional[logging.Logger] = None,
        log_level: int = logging.INFO,
    ) -> None:
        self.total_images: int = 0
        self.corrected_images: int = 0
        self.egomotion_corrections: int = 0
        self._csv_header_written: set[str] = set()
        self.config = config or IlluminationConfig()
        self.logger = logger or self._make_logger(log_level)
        self.logger.debug(f"IlluminationEnhancer initialized with config={self.config}")


    # ------------------------ Logging ---------------------------------------
    @staticmethod
    def _make_logger(level: int) -> logging.Logger:
        if setup_custom_logger is not None:
            return setup_custom_logger(level=level)
        logger = logging.getLogger("illumination")
        if not logger.handlers:
            logger.setLevel(level)
            h = logging.StreamHandler()

            # ANSI color formatter: INFO=gray, WARNING=yellow, ERROR=red, DEBUG=orange
            class _ColorFmt(logging.Formatter):
                _COL = {
                    "INFO": "\033[90m",          # gray
                    "WARNING": "\033[33m",       # yellow
                    "ERROR": "\033[31m",         # red
                    "DEBUG": "\033[38;5;208m",   # orange
                }
                _RESET = "\033[0m"

                def format(self, record: logging.LogRecord) -> str:
                    base = f"%(asctime)s - %(levelname)s - %(message)s"
                    msg = logging.Formatter(base).format(record)
                    col = self._COL.get(record.levelname, "")
                    if col:
                        return f"{col}{msg}{self._RESET}"
                    return msg

            fmt = _ColorFmt()
            h.setFormatter(fmt)
            logger.addHandler(h)
        return logger

    @staticmethod
    def _bayer_to_code(encoding: str, to_bgr: bool = True) -> int:
        """Map ROS Bayer encoding to OpenCV demosaic code."""
        enc = encoding.lower()
        if "rggb" in enc:
            return cv2.COLOR_BayerRG2BGR if to_bgr else cv2.COLOR_BayerRG2RGB
        if "bggr" in enc:
            return cv2.COLOR_BayerBG2BGR if to_bgr else cv2.COLOR_BayerBG2RGB
        if "grbg" in enc:
            return cv2.COLOR_BayerGR2BGR if to_bgr else cv2.COLOR_BayerGR2RGB
        if "gbrg" in enc:
            return cv2.COLOR_BayerGB2BGR if to_bgr else cv2.COLOR_BayerGB2RGB
        raise ValueError(f"Unsupported Bayer encoding: {encoding}")

    @staticmethod
    def remosaic_bgr(img_bgr: np.ndarray, pattern: str = "rggb") -> np.ndarray:
        """
        Convert a BGR image back to a Bayer mosaic (uint8).
        Supports RGGB, BGGR, GRBG, GBRG.
        """
        h, w = img_bgr.shape[:2]
        bayer = np.zeros((h, w), dtype=np.uint8)
        if pattern == "rggb":
            bayer[0::2, 0::2] = img_bgr[0::2, 0::2, 2]  # R
            bayer[0::2, 1::2] = img_bgr[0::2, 1::2, 1]  # G
            bayer[1::2, 0::2] = img_bgr[1::2, 0::2, 1]  # G
            bayer[1::2, 1::2] = img_bgr[1::2, 1::2, 0]  # B
        elif pattern == "bggr":
            bayer[0::2, 0::2] = img_bgr[0::2, 0::2, 0]  # B
            bayer[0::2, 1::2] = img_bgr[0::2, 1::2, 1]  # G
            bayer[1::2, 0::2] = img_bgr[1::2, 0::2, 1]  # G
            bayer[1::2, 1::2] = img_bgr[1::2, 1::2, 2]  # R
        elif pattern == "grbg":
            bayer[0::2, 0::2] = img_bgr[0::2, 0::2, 1]  # G
            bayer[0::2, 1::2] = img_bgr[0::2, 1::2, 2]  # R
            bayer[1::2, 0::2] = img_bgr[1::2, 0::2, 0]  # B
            bayer[1::2, 1::2] = img_bgr[1::2, 1::2, 1]  # G
        elif pattern == "gbrg":
            bayer[0::2, 0::2] = img_bgr[0::2, 0::2, 1]  # G
            bayer[0::2, 1::2] = img_bgr[0::2, 1::2, 0]  # B
            bayer[1::2, 0::2] = img_bgr[1::2, 0::2, 2]  # R
            bayer[1::2, 1::2] = img_bgr[1::2, 1::2, 1]  # G
        else:
            raise ValueError(f"Unsupported Bayer pattern: {pattern}")

        return bayer

    # ------------------------ Public API ------------------------------------
    
    def correct_auto(
        self,
        img: np.ndarray,
        encoding: str = "bgr8",
        force: bool = False,
        imu_msg: Optional[object] = None,
        K: Optional[np.ndarray] = None,
        R_cam_imu: Optional[np.ndarray] = None,
        exposure_time: float = 0.01,
    ) -> Tuple[np.ndarray, Dict[str, object]]:
        """Automatically pick Bayer or BGR correction based on encoding."""
        if encoding.lower().startswith("bayer_"):
            code = self._bayer_to_code(encoding, to_bgr=False)
            bgr = cv2.cvtColor(img, code)
            return self.correct_image(
                bgr,
                force=force, imu_msg=imu_msg, K=K, R_cam_imu=R_cam_imu,
                exposure_time=exposure_time, context_frames=None
            )
        else:
            return self.correct_image(
                img,
                force=force, imu_msg=imu_msg, K=K, R_cam_imu=R_cam_imu,
                exposure_time=exposure_time, context_frames=None
            )


    def correct_image(
        self,
        img_bgr: np.ndarray,
        force: bool = False,
        imu_msg: Optional[object] = None,
        K: Optional[np.ndarray] = None,
        R_cam_imu: Optional[np.ndarray] = None,
        exposure_time: float = 0.01,
        context_frames: Optional[List[np.ndarray]] = None,  # <-- new
    ) -> Tuple[np.ndarray, Dict[str, object]]:
        """Correct illumination on a single image; optionally compensate egomotion.

        If IMU + intrinsics + extrinsics are provided and the estimated rotation
        during exposure exceeds ``ego_theta_thresh_rad``, a pure-rotation warp is
        applied first (H ≈ K·R·K⁻¹). Then illumination correction decisions are
        made from robust luminance statistics.

        Returns:
            corrected image, and info dict containing:
            - action: one of 'egomotion_corrected','underexposed','overexposed',
                      'low_contrast','ok'
            - reason: string explanation
            - metrics_before, metrics_after including median and clipped stats
            - gamma_used
        """
        self.logger.debug(f"Correcting BGR image {img_bgr.shape}, force={force}, imu={'yes' if imu_msg else 'no'}")
        luma = self._compute_luminance_bgr(img_bgr)
        m_before = self._exposure_metrics_robust(luma)
        self.logger.debug(f"Metrics before: {m_before}")
        actions = []
        reasons = []
        gamma_used = 1.0

        self.total_images += 1
        ego_info= {}

        # --- Optional egomotion compensation (pure rotation) ---
        if imu_msg is not None and K is not None and R_cam_imu is not None:
            img_bgr_ego, applied, ego_info = self._correct_for_egomotion(
                img_bgr, imu_msg, K, R_cam_imu, exposure_time, self.config.ego_theta_thresh_rad
            )
            if applied:
                img_bgr = img_bgr_ego
                actions.append("egomotion")
                reasons.append("significant rotational motion during exposure")
                self.egomotion_corrections += 1
                # refresh luminance metrics after motion warp
                luma = self._compute_luminance_bgr(img_bgr)
                m_before = self._exposure_metrics_robust(luma)
            else:
                ego_info = {"applied": False}
            self.logger.debug(f"Egomotion applied={applied} info={ego_info}")
        deblur_info = {"mode": "off"}

        if self.config.deblur_enabled and self.config.deblur_mode != "off":
            if self.config.deblur_mode == "multiframe" and context_frames:
                img_bgr = self._deblur_multiframe(img_bgr, context_frames)
                actions.append("deblur_multiframe")
            else:
                img_bgr = self._deblur_single(img_bgr, imu_msg, K, exposure_time)
                actions.append("deblur_single")

        # --- Illumination decision & correction ---
        act2, reason2 = self._decide_action(m_before)
        if force or act2 != "good":
            corrected, m_after, gamma_used = self._apply_correction(img_bgr, act2, m_before)
            img_bgr = corrected
            self.corrected_images += 1
            actions.append(act2)
            if reason2:
                reasons.append(reason2)
        else:
            m_after = m_before
            if act2 == "good":
                reason2 = "within nominal bounds"
            actions.append(act2)
            reasons.append(reason2)

        info = {
            "actions": actions,
            "reasons": reasons,
            "metrics_before": m_before,
            "metrics_after": m_after,
            "gamma_used": gamma_used,
            "egomotion_info": ego_info,
        }

        self.logger.debug(f"Decision={act2}, reason={reason2}, gamma={gamma_used}")
        self.logger.debug(f"Actions={actions}, Reasons={reasons}")
        return img_bgr, info

    # ------------------------ Batch helpers ---------------------------------
    def process_bag(
        self,
        bag_path: str,
        out_bag_path: Optional[str] = None,
        topics: Optional[List[str]] = None,
        report_dir: Optional[str] = None,
        force: bool = False,
        exposure_time: float = 0.01,
        R_cam_imu_map: Optional[Dict[str, np.ndarray]] = None,
        K_intr: Optional[np.ndarray] = None,
        preserve_bayer: bool = True,
    ) -> Dict[str, int]:
        """Process a single ROS bag file and optionally write a corrected version.

        Args:
            bag_path: Path to input bag file.
            out_bag_path: Path to output bag (if None, don’t write).
            topics: List of image topics to process.
            report_dir: Directory to save per-image reports (optional).
            force: Force correction even if action='good'.
            exposure_time: Exposure time in seconds (used for egomotion).
            R_cam_imu_map: Per-topic extrinsics (optional).
            K_intr: Camera intrinsics matrix (optional, for egomotion).
            preserve_bayer: If False, save demosaiced bgr8 instead of Bayer.

        Returns:
            dict with run summary (corrected/total/egomotion counts).
        """
        import rosbag
        from cv_bridge import CvBridge

        os.makedirs(report_dir, exist_ok=True) if report_dir else None
        bridge = CvBridge()

        writer = rosbag.Bag(out_bag_path, "w") if out_bag_path else None
        summary_before = self.get_run_summary()
        ctx_per_topic: Dict[str, deque] = {t: deque(maxlen=self.config.mf_window) for t in topics}

        # --- buffer for IMU messages ---
        imu_buffer: List[object] = []

        with rosbag.Bag(bag_path, "r") as in_bag:
            total_msgs = in_bag.get_message_count(topic_filters=topics)
            corrected_count = 0
            with tqdm(total=total_msgs,
                          desc=f"Processing {os.path.basename(bag_path)}",
                          unit="msg",
                          position=1,
                          leave=False) as msg_pbar:

                for topic, msg, t in in_bag.read_messages(topics=topics):
                    msg_pbar.set_postfix_str(f"{topic}")  # show current topic
                    mtype = getattr(msg, "_type", "")

                    # --- Collect IMU messages ---
                    if "Imu" in mtype:
                        imu_buffer.append(msg)
                        if writer:
                            writer.write(topic, msg, t)
                        continue

                    # --- Handle image topics ---
                    if "Image" in mtype:
                        enc = getattr(msg, "encoding", "bgr8").lower()
                        desired = "passthrough" if enc.startswith("bayer_") else "bgr8"
                        img = bridge.imgmsg_to_cv2(msg, desired_encoding=desired)

                        # Context frames
                        ctxq = ctx_per_topic[topic]
                        context_frames = list(ctxq) if ctxq else None

                        # Interpolate IMU to image timestamp
                        imu_msg = None
                        if imu_buffer:
                            imu_msg = self._interpolate_imu(imu_buffer, t.to_sec())

                        corrected, info = self.correct_auto(
                            img, encoding=enc, force=force,
                            imu_msg=imu_msg, K=K_intr,
                            R_cam_imu=R_cam_imu_map.get(topic) if R_cam_imu_map else None,
                            exposure_time=exposure_time,
                        )

                        # push current into context AFTER correction
                        ctxq.append(img.copy())
                        # Count only real corrections
                        if force or "good" not in info["actions"]:
                            corrected_count += 1
                        # Reports
                        if report_dir and (force or "good" not in info.get("actions", [])):
                            if enc.startswith("bayer_"):
                                code = self._bayer_to_code(enc, to_bgr=False)
                                raw_vis = cv2.cvtColor(img, code)       # demosaic Bayer input
                                corr_vis = corrected                    # already BGR
                            else:
                                raw_vis = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                                corr_vis = cv2.cvtColor(corrected, cv2.COLOR_BGR2RGB)

                            report_path = os.path.join(report_dir, f"{t.to_nsec()}_{topic.replace('/','_')}_report.png")

                            self._save_report_figure(
                                img_bgr=raw_vis, corrected_bgr=corr_vis,
                                m_before=info["metrics_before"], m_after=info["metrics_after"],
                                actions=info["actions"], reasons=info["reasons"],
                                save_path=report_path,
                                ego_info=info.get("egomotion_info"),
                                csv_path=os.path.join(report_dir, "illumination_summary.csv"),
                                row_id=f"{t.to_nsec()}_{topic}",
                                topic=topic  # new argument
                            )

                        # Save corrected image back to bag
                        if writer:
                            if preserve_bayer and enc.startswith("bayer_"):
                                out_enc = enc
                            else:
                                out_enc = "bgr8"
                            out_msg = bridge.cv2_to_imgmsg(corrected, encoding=out_enc)
                            if hasattr(msg, "header"):
                                out_msg.header = msg.header
                            writer.write(topic, out_msg, t)
                    msg_pbar.update(1)
                msg_pbar.set_postfix({"topic": topic, "corrected": corrected_count})
                msg_pbar.close()
            if writer:
                writer.close()

            return self.get_run_summary()

    def to_rgb(img):
        if img.ndim == 2:  # grayscale
            return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        if img.shape[2] == 3:
            # Assume BGR unless already from Bayer demosaic (then it's RGB)
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def process_folder(
        self,
        src: str,
        dst: str,
        report_dir: Optional[str] = None,
        force: bool = False,
        suffix: str = "",
        preserve_bayer: bool = True,
    ) -> Dict[str, int]:
        """Batch process a directory of images (Bayer or BGR).

        Args:
            src: Input folder path.
            dst: Output folder path for corrected images.
            report_dir: Optional path for per-image reports (with CSV).
            force: Apply correction even if action='good'.
            suffix: Suffix to add to output filenames.
            preserve_bayer: If False, Bayer images are demosaiced to BGR8 before saving.

        Returns:
            dict with run summary (corrected/total/egomotion counts).
        """
        os.makedirs(dst, exist_ok=True)
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)

        exts = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")
        names = [n for n in sorted(os.listdir(src)) if n.lower().endswith(exts)]
        if not names:
            self.logger.warning("No images found in %s", src)
            return {}
        ctxq = deque(maxlen=self.config.mf_window)

        for name in names:
            in_path = os.path.join(src, name)
            img = cv2.imread(in_path, cv2.IMREAD_UNCHANGED)
            if img is None:
                self.logger.warning("Cannot read %s; skipping", in_path)
                continue
            context_frames = list(ctxq) if ctxq else None
            enc = "bgr8"  # TODO: detect Bayer if needed
            corrected, info = self.correct_auto(
                    img, force=force,
                    context_frames=context_frames
                )
            ctxq.append(img.copy())
            # corrected, info = self.correct_auto(img, encoding=encoding, force=force)

            if info["action"] == "good" and not force:
                continue

            # Decide how to save
            stem, ext = os.path.splitext(name)
            out_name = f"{stem}{suffix}{ext}"
            out_path = os.path.join(dst, out_name)
            if encoding.startswith("bayer_") and preserve_bayer:
                cv2.imwrite(out_path, corrected)  # still single-channel Bayer
            else:
                cv2.imwrite(out_path, corrected)  # corrected BGR8

            # Reports
            if report_dir:
                if encoding.startswith("bayer_"):
                    code = self._bayer_to_code(encoding, to_bgr=False)
                    raw_vis = cv2.cvtColor(img, code)
                    corr_vis = cv2.cvtColor(corrected, code)
                else:
                    raw_vis = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    corr_vis = cv2.cvtColor(corrected, cv2.COLOR_BGR2RGB)

                report_path = os.path.join(report_dir, f"{stem}_report.png")
                self._save_report_figure(
                    img_bgr=raw_vis,
                    corrected_bgr=corr_vis,
                    m_before=info["metrics_before"],
                    m_after=info["metrics_after"],
                    action=info["actions"], reason=info["reasons"],
                    save_path=report_path,
                    ego_info=info.get("egomotion_info"),
                    csv_path=os.path.join(report_dir, "illumination_summary.csv"),
                    row_id=stem,
                )

        self.logger.info("Finished processing folder %s → %s", src, dst)
        return self.get_run_summary()


    # ------------------------ Metrics & decisions ----------------------------
    @staticmethod
    def _compute_luminance_bgr(img_bgr: np.ndarray) -> np.ndarray:
        """Compute Y channel from BGR image (8-bit)."""
        ycrcb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YCrCb)
        return ycrcb[:, :, 0]

    def _exposure_metrics_robust(self, luma_u8: np.ndarray) -> Dict[str, float]:
        """Robust exposure metrics (median/mean/std on clipped distribution)."""
        flat = luma_u8.reshape(-1)
        low, high = np.percentile(flat, [0.5, 99.5])
        clipped = flat[(flat >= low) & (flat <= high)]
        if clipped.size == 0:
            clipped = flat

        median_clipped = float(np.median(clipped))
        mean_clipped = float(np.mean(clipped))
        std_clipped = float(np.std(clipped))
        p1 = float(np.percentile(flat, 1))
        p99 = float(np.percentile(flat, 99))
        dyn_range = p99 - p1
        pct_dark = float((flat <= 20).sum()) * 100.0 / flat.size
        pct_bright = float((flat >= 235).sum()) * 100.0 / flat.size
        self.logger.debug(f"Exposure metrics: mean={mean_clipped:.2f}, \
            median={median_clipped:.2f}, dyn_range={dyn_range:.2f}")


        return {
            "median_clipped": median_clipped,
            "mean_clipped": mean_clipped,
            "std_clipped": std_clipped,
            "pct_dark": pct_dark,
            "pct_bright": pct_bright,
            "p1": p1,
            "p99": p99,
            "dyn_range": dyn_range,
        }

    def _decide_action(self, m: Dict[str, float]) -> Tuple[str, str]:
        """Decide action label & reason using robust stats."""
        cfg = self.config
        reasons: List[str] = []
        action = "good"

        if (m["mean_clipped"] < cfg.mean_dark) or (m["pct_dark"] > cfg.pct_dark_th):
            action = "underexposed"
            if m["mean_clipped"] < cfg.mean_dark:
                reasons.append(f"low mean (clipped) {m['mean_clipped']:.1f}")
            if m["pct_dark"] > cfg.pct_dark_th:
                reasons.append(f"{m['pct_dark']:.1f}% pixels very dark")
        elif (m["mean_clipped"] > cfg.mean_bright) or (m["pct_bright"] > cfg.pct_bright_th):
            action = "overexposed"
            if m["mean_clipped"] > cfg.mean_bright:
                reasons.append(f"high mean (clipped) {m['mean_clipped']:.1f}")
            if m["pct_bright"] > cfg.pct_bright_th:
                reasons.append(f"{m['pct_bright']:.1f}% clipped bright")

        if action == "good" and m["dyn_range"] < cfg.dyn_range_min:
            action = "low_contrast"
            reasons.append(f"narrow dyn_range {m['dyn_range']:.1f}")

        reason = ", ".join(reasons) if reasons else "within nominal bounds"
        self.logger.debug(f"Decision: action={action}, reason={reason}")

        return action, reason

    # ------------------------ Corrections ------------------------------------

        
    def _apply_correction(
        self, img_bgr: np.ndarray, action: str, metrics_before: Dict[str, float]
    ) -> Tuple[np.ndarray, Dict[str, float], float]:
        """Apply WB/CLAHE/gamma based on action."""
        cfg = self.config
        out = img_bgr.copy()
        gamma_used = 1.0

        if cfg.white_balance:
            out = self._gray_world_wb(out)

        
        if action in ("underexposed", "low_contrast"):
            if cfg.under_method == "dynamic":
                # existing dynamic CLAHE + gamma
                pct_dark = metrics_before.get("pct_dark", 0.0)
                strength = np.clip(pct_dark / 50.0, 0.0, 1.0)
                out = self._apply_clahe_on_l(out, clip_limit=cfg.clahe_clip_limit * (0.5 + strength),
                                             tiles=cfg.clahe_tiles)
                gamma_est = self._estimate_gamma_for_target_mean(
                    metrics_before["median_clipped"],
                    cfg.target_mean_under,
                    cfg.gamma_min,
                    cfg.gamma_max,
                )
                gamma_used = 1.0 + (gamma_est - 1.0) * strength
                out = self._gamma_correct(out, gamma_used)
            else:
                out = self._correct_underexposed(out, cfg.under_method)

        elif action == "overexposed":
            if cfg.over_method == "dynamic":
                # existing dynamic luminance-only correction
                pct_clip = metrics_before.get("pct_bright", 0.0)
                strength = np.clip(pct_clip / 50.0, 0.0, 1.0)
                lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
                L, A, B = cv2.split(lab)
                L = L.astype(np.float32)
                mask = L > 220
                L[mask] = 220 + (L[mask] - 220) * (1.0 - 0.7 * strength)
                L = np.clip(L, 0, 255).astype(np.uint8)
                gamma_used = 1.0 + 0.3 * strength
                inv = 1.0 / 255.0
                lut = np.array([((i * inv) ** gamma_used) * 255.0 for i in range(256)], dtype=np.uint8)
                L = cv2.LUT(L, lut)
                clahe_limit = 1.0 + 1.5 * strength
                clahe = cv2.createCLAHE(clipLimit=clahe_limit, tileGridSize=(cfg.clahe_tiles, cfg.clahe_tiles))
                L = clahe.apply(L)
                lab = cv2.merge((L, A, B))
                out = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            else:
                out = self._correct_overexposed(out, cfg.over_method)


        m_after = self._exposure_metrics_robust(self._compute_luminance_bgr(out))
        self.logger.debug(f"Applying correction: action={action}, gamma={gamma_used}")

        return out, m_after, gamma_used


    def _correct_overexposed(self, img_bgr: np.ndarray, method: str) -> np.ndarray:
        img_f32 = img_bgr.astype(np.float32) / 255.0

        if method == "reinhard":
            tonemap = cv2.createTonemapReinhard(gamma=1.0, intensity=0.0, light_adapt=0.8, color_adapt=0.0)
            out = tonemap.apply(img_f32)
            return np.clip(out * 255, 0, 255).astype(np.uint8)

        elif method == "drago":
            tonemap = cv2.createTonemapDrago(gamma=1.0, saturation=1.0, bias=0.9)
            out = tonemap.apply(img_f32)
            return np.clip(out * 255, 0, 255).astype(np.uint8)

        elif method == "mantiuk":
            tonemap = cv2.createTonemapMantiuk(gamma=1.0, scale=0.85, saturation=1.0)
            out = tonemap.apply(img_f32)
            return np.clip(out * 255, 0, 255).astype(np.uint8)

        elif method == "clahe":
            return self._apply_clahe_on_l(img_bgr, clip_limit=2.0, tiles=8)

        elif method == "msr":
            return self._retinex_msr(img_bgr)

        elif method == "lime":
            return self._lime_enhance(img_bgr)

        return img_bgr


    def _correct_underexposed(self, img_bgr: np.ndarray, method: str) -> np.ndarray:
        if method == "clahe":
            return self._apply_clahe_on_l(img_bgr, clip_limit=2.0, tiles=8)

        elif method == "gamma":
            return self._gamma_correct(img_bgr, gamma=0.7)

        elif method == "msr":
            return self._retinex_msr(img_bgr)

        elif method == "lime":
            return self._lime_enhance(img_bgr)

        elif method == "dynamic":
            # fall back to your existing CLAHE + gamma scaling
            return img_bgr

        return img_bgr


    def _retinex_msr(self, img_bgr: np.ndarray, scales: List[int] = [15, 80, 250]) -> np.ndarray:
        img = img_bgr.astype(np.float32) + 1.0
        log_img = np.log(img)
        msr = np.zeros_like(img)
        for scale in scales:
            blur = cv2.GaussianBlur(img, (0, 0), sigmaX=scale, sigmaY=scale)
            msr += (log_img - np.log(blur + 1.0))
        msr /= len(scales)
        msr = cv2.normalize(msr, None, 0, 255, cv2.NORM_MINMAX)
        return msr.astype(np.uint8)


    def _lime_enhance(self, img_bgr: np.ndarray) -> np.ndarray:
        img = img_bgr.astype(np.float32) / 255.0
        # Estimate illumination map: max across channels
        illum = np.max(img, axis=2)
        illum = cv2.ximgproc.guidedFilter(guide=illum.astype(np.float32), src=illum.astype(np.float32),
                                          radius=15, eps=1e-3)
        illum = np.expand_dims(illum, axis=2)
        enhanced = img / (illum + 1e-6)
        enhanced = np.clip(enhanced, 0, 1)
        return (enhanced * 255).astype(np.uint8)



    # ---------- Motion PSF from IMU ----------
    @staticmethod
    def _interpolate_imu(imu_msgs: List[object], t_sec: float) -> Optional[object]:
        """
        Interpolate IMU angular velocity & linear acceleration to a given timestamp.

        Args:
            imu_msgs: list of sensor_msgs/Imu (time-sorted)
            t_sec: target timestamp (float seconds)

        Returns:
            Interpolated Imu message (deepcopy of nearest), or None if no messages.
        """
        if not imu_msgs:
            return None

        imu_times = [m.header.stamp.to_sec() for m in imu_msgs]

        # Before first or after last sample → nearest neighbor
        if t_sec <= imu_times[0]:
            return deepcopy(imu_msgs[0])
        if t_sec >= imu_times[-1]:
            return deepcopy(imu_msgs[-1])

        # Binary search for surrounding indices
        import bisect
        idx = bisect.bisect_left(imu_times, t_sec)
        i0, i1 = idx - 1, idx
        t0, t1 = imu_times[i0], imu_times[i1]
        m0, m1 = imu_msgs[i0], imu_msgs[i1]

        alpha = (t_sec - t0) / (t1 - t0 + 1e-9)

        imu_msg = deepcopy(m0)

        # Interpolate angular velocity
        imu_msg.angular_velocity.x = (1 - alpha) * m0.angular_velocity.x + alpha * m1.angular_velocity.x
        imu_msg.angular_velocity.y = (1 - alpha) * m0.angular_velocity.y + alpha * m1.angular_velocity.y
        imu_msg.angular_velocity.z = (1 - alpha) * m0.angular_velocity.z + alpha * m1.angular_velocity.z

        # Interpolate linear acceleration
        imu_msg.linear_acceleration.x = (1 - alpha) * m0.linear_acceleration.x + alpha * m1.linear_acceleration.x
        imu_msg.linear_acceleration.y = (1 - alpha) * m0.linear_acceleration.y + alpha * m1.linear_acceleration.y
        imu_msg.linear_acceleration.z = (1 - alpha) * m0.linear_acceleration.z + alpha * m1.linear_acceleration.z

        # Assign target timestamp
        imu_msg.header.stamp = imu_msgs[i0].header.stamp.__class__.from_sec(t_sec)

        return imu_msg
    @staticmethod
    def _motion_kernel(length_px: float, angle_rad: float, width: float = 1.0) -> np.ndarray:
        """Create a normalized linear motion blur kernel (PSF) of given length and angle."""
        length = max(1, int(round(length_px)))
        # Make a small square kernel that fully contains the line
        size = max(3, length + 2)
        k = np.zeros((size, size), np.float32)
        # Draw an anti-aliased line through the center at angle
        center = (size - 1) / 2.0
        dx = np.cos(angle_rad)
        dy = np.sin(angle_rad)
        # sample points along the line
        num = max(length * 4, 8)
        for i in range(num):
            t = (i / (num - 1)) * (length - 1) - (length - 1) / 2.0
            x = center + dx * t
            y = center + dy * t
            xi, yi = int(round(x)), int(round(y))
            if 0 <= yi < size and 0 <= xi < size:
                k[yi, xi] += 1.0
        k = cv2.GaussianBlur(k, (0, 0), width)
        s = k.sum()
        if s > 0:
            k /= s
        return k

    def _estimate_psf_from_imu(self, imu_msg, exposure_time: float, K: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """
        Estimate a small linear PSF from angular rates during exposure.
        Assumes pure rotation; translation is ignored (needs depth).
        """
        if imu_msg is None or exposure_time is None or K is None:
            return None
        # angular velocity (rad/s)
        wx = getattr(imu_msg.angular_velocity, "x", 0.0)
        wy = getattr(imu_msg.angular_velocity, "y", 0.0)
        # integrate over exposure (small angle)
        thetax = float(wx) * float(exposure_time)
        thetay = float(wy) * float(exposure_time)

        # Approximate pixel shift ≈ f * theta (for yaw/pitch)
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        shift_x = fx * thetay   # yaw rotates image horizontally
        shift_y = fy * thetax   # pitch rotates image vertically
        length_px = np.hypot(shift_x, shift_y)
        if length_px < self.config.deblur_psf_min_px:
            return None
        length_px = float(np.clip(length_px, self.config.deblur_psf_min_px, self.config.deblur_psf_max_px))
        angle = np.arctan2(shift_y, shift_x + 1e-9)
        return self._motion_kernel(length_px, angle, width=0.8)

    # ---------- Single-frame deconvolution ----------

    @staticmethod
    def _richardson_lucy(img_gray: np.ndarray, psf: np.ndarray, iters: int) -> np.ndarray:
        """Richardson–Lucy deconvolution on [0,1] grayscale."""
        eps = 1e-6
        I = img_gray.astype(np.float32).clip(0, 1)
        P = psf.astype(np.float32)
        P = P / (P.sum() + eps)
        est = I.copy()
        P_flip = cv2.flip(P, -1)
        for _ in range(max(1, iters)):
            conv = cv2.filter2D(est, -1, P, borderType=cv2.BORDER_REPLICATE).clip(eps, 1.0)
            relative = (I / conv)
            est *= cv2.filter2D(relative, -1, P_flip, borderType=cv2.BORDER_REPLICATE)
            est = est.clip(0, 1)
        return est

    @staticmethod
    def _wiener_deconv(img_gray: np.ndarray, psf: np.ndarray, K: float) -> np.ndarray:
        """Simple frequency-domain Wiener deconvolution on [0,1] grayscale."""
        eps = 1e-6
        I = img_gray.astype(np.float32).clip(0, 1)
        P = psf.astype(np.float32)
        P /= (P.sum() + eps)
        # pad psf to image size
        h, w = I.shape
        ph, pw = P.shape
        pad = np.zeros((h, w), np.float32)
        pad[:ph, :pw] = P
        # shift PSF to center
        pad = np.roll(np.roll(pad, -ph//2, axis=0), -pw//2, axis=1)

        FI = np.fft.rfft2(I)
        FP = np.fft.rfft2(pad)
        denom = (np.abs(FP)**2 + K)
        out = np.fft.irfft2(FI * np.conj(FP) / np.maximum(denom, eps), s=I.shape)
        return np.clip(out, 0, 1).astype(np.float32)

    def _deblur_single(self, img_bgr: np.ndarray, imu_msg, K: Optional[np.ndarray], exposure_time: float) -> np.ndarray:
        """Single-frame deblur using IMU-estimated PSF and RL/Wiener on luminance."""
        psf = self._estimate_psf_from_imu(imu_msg, exposure_time, K)
        if psf is None:
            return img_bgr
        # work in LAB luminance to preserve color
        lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
        L = lab[:, :, 0].astype(np.float32) / 255.0
        if self.config.deblur_algorithm == "wiener":
            Ld = self._wiener_deconv(L, psf, self.config.deblur_wiener_k)
        else:
            Ld = self._richardson_lucy(L, psf, self.config.deblur_iters)
        lab[:, :, 0] = np.clip(Ld * 255.0, 0, 255).astype(np.uint8)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # ---------- Multi-frame fusion (flow or egomotion) ----------

    @staticmethod
    def _sharpness_weight(gray: np.ndarray, metric: str = "laplacian") -> np.ndarray:
        g = gray.astype(np.float32) / 255.0
        if metric == "sobel":
            sx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
            sy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
            m = cv2.magnitude(sx, sy)
        else:
            m = cv2.Laplacian(g, cv2.CV_32F, ksize=3)
            m = np.abs(m)
        m = cv2.GaussianBlur(m, (0, 0),  self.config.mf_sigma)
        m = np.maximum(m, 1e-6)
        return m

    def _align_by_flow(self, ref_gray: np.ndarray, nbr_bgr: np.ndarray) -> np.ndarray:
        nbr_gray = cv2.cvtColor(nbr_bgr, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(nbr_gray, ref_gray,
                                            None, 0.5, 3, 15, 3, 5, 1.2, 0)
        h, w = ref_gray.shape
        grid_x, grid_y = np.meshgrid(np.arange(w), np.arange(h))
        map_x = (grid_x + flow[..., 0]).astype(np.float32)
        map_y = (grid_y + flow[..., 1]).astype(np.float32)
        return cv2.remap(nbr_bgr, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)

    def _align_by_ego(self, ref_bgr: np.ndarray, nbr_bgr: np.ndarray) -> np.ndarray:
        """Hook: if you already compute homographies from egomotion, call that here.
        Default falls back to optical flow."""
        ref_gray = cv2.cvtColor(ref_bgr, cv2.COLOR_BGR2GRAY)
        return self._align_by_flow(ref_gray, nbr_bgr)

    def _deblur_multiframe(self, center_bgr: np.ndarray, context_frames: List[np.ndarray]) -> np.ndarray:
        """Fuse aligned neighbors using sharpness weights."""
        if not context_frames:
            return center_bgr
        ref_gray = cv2.cvtColor(center_bgr, cv2.COLOR_BGR2GRAY)
        aligned = [center_bgr]
        for nbr in context_frames:
            if self.config.mf_align == "ego":
                aligned.append(self._align_by_ego(center_bgr, nbr))
            else:
                aligned.append(self._align_by_flow(ref_gray, nbr))
        # sharpness weights per frame
        weights = [self._sharpness_weight(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), self.config.mf_sharpness_metric) for f in aligned]
        W = np.stack(weights, axis=-1)  # HxWxN
        Wsum = np.sum(W, axis=-1, keepdims=True) + 1e-6
        Wn = W / Wsum
        # weighted fusion
        acc = np.zeros_like(center_bgr, dtype=np.float32)
        for i, f in enumerate(aligned):
            acc += f.astype(np.float32) * Wn[..., i:i+1]
        return np.clip(acc, 0, 255).astype(np.uint8)
    @staticmethod
    def _gray_world_wb(img_bgr: np.ndarray) -> np.ndarray:
        """Gray-world white balance."""
        img = img_bgr.astype(np.float32)
        mean_b = float(img[:, :, 0].mean()) + 1e-6
        mean_g = float(img[:, :, 1].mean()) + 1e-6
        mean_r = float(img[:, :, 2].mean()) + 1e-6
        gray = (mean_b + mean_g + mean_r) / 3.0
        gains = (gray / mean_b, gray / mean_g, gray / mean_r)
        out = img * np.array(gains, dtype=np.float32)[None, None, :]
        out = np.clip(out, 0, 255).astype(np.uint8)
        return out

    @staticmethod
    def _apply_clahe_on_l(img_bgr: np.ndarray, clip_limit: float, tiles: int) -> np.ndarray:
        """Apply CLAHE on L channel in LAB space."""
        lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tiles, tiles))
        l2 = clahe.apply(l)
        lab2 = cv2.merge((l2, a, b))
        return cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)

    @staticmethod
    def _gamma_correct(img_bgr: np.ndarray, gamma: float) -> np.ndarray:
        """Apply gamma correction via LUT."""
        g = max(float(gamma), 1e-3)
        inv255 = 1.0 / 255.0
        lut = np.array([((i * inv255) ** (g)) * 255.0 for i in range(256)], dtype=np.uint8)
        return cv2.LUT(img_bgr, lut)

    @staticmethod
    def _estimate_gamma_for_target_mean(curr_mean: float, target: float, gmin: float, gmax: float) -> float:
        """Estimate gamma to move median brightness toward a target, clamped."""
        curr = max(curr_mean, 1.0)
        ratio = target / curr
        gamma = ratio ** 0.7
        return float(np.clip(gamma, gmin, gmax))

    # ------------------------ Egomotion (pure rotation) ----------------------
    def _correct_for_egomotion(
        self,
        img_bgr: np.ndarray,
        imu_msg: object,
        K: np.ndarray,
        R_cam_imu: np.ndarray,
        exposure_time: float,
        theta_thresh: float,
    ) -> Tuple[np.ndarray, bool, Dict[str, float]]:
        """Compensate pure rotational egomotion during exposure using IMU.

        Assumes small-angle rotation ΔR ≈ exp([ω_cam]×·Δt). The corresponding
        image warp for pure rotation is H ≈ K·ΔR·K⁻¹ (conjugate rotation).

        Returns the possibly corrected image and a boolean indicating whether
        correction was applied.
        """
        # 1) Angular velocity from IMU (rad/s)
        omega_imu = np.array(
            [
                float(getattr(imu_msg.angular_velocity, "x", 0.0)),
                float(getattr(imu_msg.angular_velocity, "y", 0.0)),
                float(getattr(imu_msg.angular_velocity, "z", 0.0)),
            ],
            dtype=float,
        )
        # 2) Transform to camera frame
        omega_cam = R_cam_imu @ omega_imu
        omega_norm = float(np.linalg.norm(omega_cam))
        theta = omega_norm * float(exposure_time)
        if not np.isfinite(theta) or theta < theta_thresh:
            return img_bgr, False, {"applied": False, "theta_rad": float(theta)}

        # 3) Small rotation via Rodrigues
        axis = omega_cam / (omega_norm + 1e-12)
        R_delta, _ = cv2.Rodrigues(axis * theta)

        # 4) Homography H = K·RΔ·K⁻¹
        H = K @ R_delta @ np.linalg.inv(K)

        # 5) Warp
        h, w = img_bgr.shape[:2]
        corrected = cv2.warpPerspective(
            img_bgr, H, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT
        )

        return corrected, True, {
                "applied": True,
                "theta_rad": float(theta),
                "omega_norm": float(omega_norm),
            }
    # ------------------------------ Reports ---------------------------------
    def append_csv_row(self, csv_path: str, row_id: str,
                       m_before: Dict[str, float], m_after: Dict[str, float],
                       info: Dict[str, object]) -> None:
        """Public helper: append a CSV row (use this from your bag loop)."""
        self._append_csv(csv_path, row_id, m_before, m_after,
                         info.get("actions", []),
                         info.get("reasons", []),
                         info.get("egomotion_info"))

    def _append_csv(self, csv_path: str, row_id: str,
                    m_before: Dict[str, float], m_after: Dict[str, float],
                    actions: list, reasons: list,
                    ego_info: Optional[Dict[str, float]],
                    topic: Optional[str] = None) -> None:
        header = [
            "id_or_timestamp",
            "topic",
            "actions",
            "reasons",
            "before_mean", "before_median", "before_std", "before_dyn", "before_pct_dark", "before_pct_bright",
            "after_mean", "after_median", "after_std", "after_dyn",
            "egomotion_applied", "egomotion_theta_rad",
        ]
        action_str = "+".join(actions if isinstance(actions, list) else [actions])
        reason_str = " | ".join(reasons if isinstance(reasons, list) else [reasons])

        need_header = csv_path not in self._csv_header_written and not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if need_header:
                w.writerow(header)
                self._csv_header_written.add(csv_path)
            w.writerow([
                row_id,
                topic or "",   # <--- FIX: include topic
                action_str,
                reason_str,
                m_before.get("mean_clipped", ""), m_before.get("median_clipped", ""), m_before.get("std_clipped", ""),
                m_before.get("dyn_range", ""), m_before.get("pct_dark", ""), m_before.get("pct_bright", ""),
                m_after.get("mean_clipped", ""), m_after.get("median_clipped", ""), m_after.get("std_clipped", ""),
                m_after.get("dyn_range", ""),
                int(bool(ego_info.get("applied"))) if ego_info else 0,
                float(ego_info.get("theta_rad", 0.0)) if ego_info else 0.0,
            ])

    def get_run_summary(self) -> Dict[str, int]:
        """Return a small info summary: how many images corrected and with egomotion."""
        return {
            "total_images": self.total_images,
            "corrected_images": self.corrected_images,
            "egomotion_corrections": self.egomotion_corrections,
        }

    def _save_report_figure(
        self,
        img_bgr: np.ndarray,
        corrected_bgr: np.ndarray,
        m_before: Dict[str, float],
        m_after: Dict[str, float],
        actions: list,
        reasons: list,
        save_path: str,
        topic: Optional[str] = None,   # <--- make optional
        force: Optional[bool] = False,
        ego_info: Optional[Dict[str, float]] = None,
        csv_path: Optional[str] = None,
        row_id: Optional[str] = None,
    ) -> None:
        if plt is None:
            self.logger.warning("matplotlib not available; skipping report for %s", save_path)
            return

        ego_applied = bool(ego_info.get("applied")) if ego_info else False
        l_before = self._compute_luminance_bgr(img_bgr).astype(np.float32) / 255.0
        l_after = self._compute_luminance_bgr(corrected_bgr).astype(np.float32) / 255.0

        # Titles
        action_str = " + ".join(actions) if isinstance(actions, list) else str(actions)
        reason_str = " | ".join(reasons) if isinstance(reasons, list) else str(reasons)

        # Prepare data for exposure maps
        under_b = np.clip((0.4 - l_before) / 0.4, 0, 1)
        over_b = np.clip((l_before - 0.9) / 0.1, 0, 1)
        heat_before = np.stack([over_b, np.zeros_like(over_b), under_b], axis=-1)

        under_a = np.clip((0.4 - l_after) / 0.4, 0, 1)
        over_a = np.clip((l_after - 0.9) / 0.1, 0, 1)
        heat_after = np.stack([over_a, np.zeros_like(over_a), under_a], axis=-1)

        # Build figure
        nrows = 3 if ego_applied else 2
        fig, axes = plt.subplots(nrows, 3, figsize=(12, 4 * nrows))
        axes = np.atleast_2d(axes)

        # Row 0: Original + histogram + exposure map (before)
        ax1, ax2, ax3 = axes[0]
        img_vis = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB) if img_bgr.ndim == 3 else img_bgr
        ax1.imshow(img_vis)
        ax1.set_title("Original")
        ax1.axis("off")

        vals_before = (l_before * 255).flatten()
        ax2.hist(vals_before, bins=64, range=(0, 255), density=True,
                 color="gray", alpha=0.6, label="before")
        ax2.set_title("Luminance histogram (before)")
        ax2.set_xlim(0, 255)
        ax2.set_ylabel("Density")
        ax2.legend()

        ax3.imshow(heat_before)
        ax3.set_title("Exposure hotspots (before)")
        ax3.axis("off")

        # Row 1: Corrected + histogram + exposure map (after)
        ax4, ax5, ax6 = axes[1]
        ax4.imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB))
        ax4.set_title("Corrected")
        ax4.axis("off")

        vals_after = (l_after * 255).flatten()
        ax5.hist(vals_after, bins=64, range=(0, 255), density=True,
                 color="black", alpha=0.6, label="after")
        ax5.set_title("Luminance histogram (after)")
        ax5.set_xlim(0, 255)
        ax5.set_ylabel("Density")
        ax5.legend()

        ax6.imshow(heat_after)
        ax6.set_title("Exposure hotspots (after)")
        ax6.axis("off")

        # Optional Row 2: Egomotion correction view
        if ego_applied:
            ax7, ax8, ax9 = axes[2]
            # Original vs corrected overlay
            ax7.imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            ax7.set_title("Original (for motion)")
            ax7.axis("off")

            ax8.imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB))
            ax8.set_title("Ego-compensated")
            ax8.axis("off")

            diff = cv2.cvtColor(cv2.absdiff(corrected_bgr, img_bgr),
                                cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
            im = ax9.imshow(diff, cmap="inferno")
            ax9.set_title("Motion magnitude proxy")
            ax9.axis("off")
            fig.colorbar(im, ax=ax9, fraction=0.046, pad=0.04)

        # Super title
        theta_deg = float(ego_info.get("theta_rad", 0.0)) * 180.0 / np.pi if ego_applied else 0.0
        sup = f"[{topic or 'unknown_topic'}] Actions: {action_str} | Reasons: {reason_str}"
        if ego_applied:
            sup += f" | θ≈{theta_deg:.2f}°"
        fig.suptitle(sup, fontsize=12)

        plt.tight_layout(rect=[0, 0, 1, 0.95])

        # Save JPEG safely
        jpg_path = os.path.splitext(save_path)[0] + ".jpg"
        fig.savefig(jpg_path, dpi=100, format="jpeg")
        plt.close(fig)

        # Append CSV row
        if csv_path is not None and (force or "good" not in actions):
            self._append_csv(
                csv_path, row_id or pathlib.Path(save_path).stem,
                m_before, m_after, actions, reasons, ego_info,
                topic=topic
            )