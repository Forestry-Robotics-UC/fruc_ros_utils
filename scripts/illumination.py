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

import csv
import logging
import os
import pathlib
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

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
    def correct_raw_bayer(self, raw_u8: np.ndarray, force: bool = False, pattern: str = "rggb"):
        """
        Better RAW handling: demosaic → filter (color pipeline) → remosaic.
        raw_u8: single-channel Bayer mosaic (uint8).
        pattern: Bayer pattern string (rggb, bggr, grbg, gbrg).
        """
        self.logger.debug(f"Correcting RAW Bayer with demosaic/remosaic, shape={raw_u8.shape}, pattern={pattern}")

        # 1) Demosaic (default RGGB unless caller knows better)
        if pattern == "rggb":
            bgr = cv2.cvtColor(raw_u8, cv2.COLOR_BayerRG2BGR)
        elif pattern == "bggr":
            bgr = cv2.cvtColor(raw_u8, cv2.COLOR_BayerBG2BGR)
        elif pattern == "grbg":
            bgr = cv2.cvtColor(raw_u8, cv2.COLOR_BayerGR2BGR)
        elif pattern == "gbrg":
            bgr = cv2.cvtColor(raw_u8, cv2.COLOR_BayerGB2BGR)
        else:
            raise ValueError(f"Unsupported Bayer pattern: {pattern}")

        # 2) Compute luminance + decision
        luma = self._compute_luminance_bgr(bgr)
        m_before = self._exposure_metrics_robust(luma)
        self.logger.debug(f"RAW metrics before: {m_before}")
        action, reason = self._decide_action(m_before)

        if not force and action == "good":
            return raw_u8, {
                "action": "good",
                "reason": "within nominal bounds",
                "metrics_before": m_before,
                "metrics_after": m_before,
                "gamma_used": 1.0,
                "egomotion_info": {"applied": False},
            }

        # 3) Apply the same correction pipeline as color
        corrected_bgr, m_after, gamma_used = self._apply_correction(bgr, action, m_before)

        # 4) Re-mosaic to original Bayer pattern
        out_bayer = self.remosaic_bgr(corrected_bgr, pattern=pattern)

        return out_bayer, {
            "action": action,
            "reason": reason,
            "metrics_before": m_before,
            "metrics_after": m_after,
            "gamma_used": gamma_used,
            "egomotion_info": {"applied": False},
        }




    def correct_image(
        self,
        img_bgr: np.ndarray,
        force: bool = False,
        # Egomotion inputs (optional):
        imu_msg: Optional[object] = None,
        K: Optional[np.ndarray] = None,
        R_cam_imu: Optional[np.ndarray] = None,
        exposure_time: float = 0.01,
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
        action = "good"  # renamed from 'ok'
        reason = ""
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
                action = "egomotion_corrected"
                reason = "significant rotational motion during exposure"
                self.egomotion_corrections += 1
                # refresh luminance metrics after motion warp
                luma = self._compute_luminance_bgr(img_bgr)
                m_before = self._exposure_metrics_robust(luma)
            else:
                ego_info = {"applied": False}
            self.logger.debug(f"Egomotion applied={applied} info={ego_info}")

        # --- Illumination decision & correction ---
        act2, reason2 = self._decide_action(m_before)
        if force or act2 != "good":
            corrected, m_after, gamma_used = self._apply_correction(img_bgr, act2, m_before)
            if action == "good":
                action = act2
                reason = reason2
            img_bgr = corrected
            self.corrected_images += 1
        else:
            m_after = m_before
            if action == "good":
                reason = "within nominal bounds"

        info = {
            "action": action,
            "action_compat": "good" if action == "good" else action,  # legacy compatibility
            "reason": reason,
            "metrics_before": m_before,
            "metrics_after": m_after,
            "gamma_used": gamma_used,
            "egomotion_info": ego_info,
        }
        self.logger.debug(f"Decision={action}, reason={reason}, gamma={gamma_used}")
        self.logger.debug(f"Metrics after: {m_after}")
        return img_bgr, info

    # ------------------------ Batch helpers ---------------------------------
    def process_folder(
        self,
        src: str,
        dst: str,
        report_dir: Optional[str] = None,
        force: bool = False,
        suffix: str = "",
    ) -> None:
        """Batch process a directory of image files; save only changed images/reports."""
        self.logger.debug(f"Processing folder {src} -> {dst}, {len(names)} images found")

        os.makedirs(dst, exist_ok=True)
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)

        exts = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")
        names = [n for n in sorted(os.listdir(src)) if n.lower().endswith(exts)]
        if not names:
            self.logger.warning("No images found in %s", src)
            return

        for name in names:
            in_path = os.path.join(src, name)
            img = cv2.imread(in_path, cv2.IMREAD_COLOR)
            if img is None:
                self.logger.warning("Cannot read %s; skipping", in_path)
                continue

            corrected, info = self.correct_image(img, force=force)
            if info["action"] == "good" and not force:
                continue

            stem, ext = os.path.splitext(name)
            out_name = f"{stem}{suffix}{ext}"
            out_path = os.path.join(dst, out_name)
            cv2.imwrite(out_path, corrected)

            if report_dir:
                report_path = os.path.join(report_dir, f"{stem}_report.png")
                self._save_report_figure(
                    img_bgr=img,
                    corrected_bgr=corrected,
                    m_before=info["metrics_before"],
                    m_after=info["metrics_after"],
                    action=str(info["action"]),
                    reason=str(info["reason"]),
                    save_path=report_path,
                )

        self.logger.debug(f"Processing {in_path}, action={info['action']}, reason={info['reason']}")


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
    def remosaic_bgr(img_bgr: np.ndarray, pattern: str = "rggb") -> np.ndarray:
        """
        Convert a BGR image back to a Bayer mosaic (uint8).
        pattern: one of "rggb", "bggr", "grbg", "gbrg"
        """
        h, w = img_bgr.shape[:2]
        bayer = np.zeros((h, w), dtype=np.uint8)

        if pattern == "rggb":
            bayer[0::2, 0::2] = img_bgr[0::2, 0::2, 2]  # R
            bayer[0::2, 1::2] = img_bgr[0::2, 1::2, 1]  # G
            bayer[1::2, 0::2] = img_bgr[1::2, 0::2, 1]  # G
            bayer[1::2, 1::2] = img_bgr[1::2, 1::2, 0]  # B
        # add other patterns as needed
        return bayer
        
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
            out = self._apply_clahe_on_l(out, cfg.clahe_clip_limit, cfg.clahe_tiles)
            gamma_used = self._estimate_gamma_for_target_mean(
                metrics_before["median_clipped"], cfg.target_mean_under, cfg.gamma_min, cfg.gamma_max
            )
            out = self._gamma_correct(out, gamma_used)
        elif action == "overexposed":
            gamma_used = self._estimate_gamma_for_target_mean(
                metrics_before["median_clipped"], cfg.target_mean_over, cfg.gamma_min, cfg.gamma_max
            )
            if gamma_used < 1.0:
                gamma_used = max(1.15, gamma_used)
            out = self._gamma_correct(out, gamma_used)
            out = self._apply_clahe_on_l(out, min(cfg.clahe_clip_limit, 2.0), cfg.clahe_tiles)

        m_after = self._exposure_metrics_robust(self._compute_luminance_bgr(out))
        self.logger.debug(f"Applying correction: action={action}, gamma={gamma_used}")

        return out, m_after, gamma_used

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
                         info.get("action", ""), info.get("reason", ""),
                         info.get("egomotion_info"))

    def _append_csv(self, csv_path: str, row_id: str,
                    m_before: Dict[str, float], m_after: Dict[str, float],
                    action: str, reason: str,
                    ego_info: Optional[Dict[str, float]]) -> None:
        header = [
            "id_or_timestamp",
            "action",
            "reason",
            "before_mean", "before_median", "before_std", "before_dyn", "before_pct_dark", "before_pct_bright",
            "after_mean", "after_median", "after_std", "after_dyn",
            "egomotion_applied", "egomotion_theta_rad",
        ]
        need_header = csv_path not in self._csv_header_written and not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if need_header:
                w.writerow(header)
                self._csv_header_written.add(csv_path)
            w.writerow([
                row_id,
                action,
                reason,
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
        action: str,
        reason: str,
        save_path: str,
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

        fig = plt.figure(figsize=(12, 8))
        if ego_applied and ego_info is not None:
            # Movement-focused layout
            gs = fig.add_gridspec(2, 3, height_ratios=[1, 1])

            ax1 = fig.add_subplot(gs[0, 0])
            ax1.imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            ax1.set_title("Original")
            ax1.axis("off")

            ax4 = fig.add_subplot(gs[1, 0])
            ax4.imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB))
            ax4.set_title("Corrected (ego-compensated)")
            ax4.axis("off")

            diff = cv2.cvtColor(cv2.absdiff(corrected_bgr, img_bgr), cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
            ax2 = fig.add_subplot(gs[:, 1])
            im = ax2.imshow(diff, cmap="inferno")
            ax2.set_title("Apparent motion magnitude (proxy)")
            ax2.axis("off")
            fig.colorbar(im, ax=ax2, fraction=0.046, pad=0.04)

            ax3 = fig.add_subplot(gs[:, 2])
            ax3.hist((l_before * 255.0).flatten(), bins=64, range=(0, 255), color="gray", alpha=0.6, label="before")
            ax3.hist((l_after * 255.0).flatten(), bins=64, range=(0, 255), color="black", alpha=0.6, label="after")
            ax3.set_title("Luminance hist (before/after)")
            ax3.set_xlim(0, 255)
            ax3.legend()

            theta_deg = float(ego_info.get("theta_rad", 0.0)) * 180.0 / 3.1415926535
            fig.suptitle(f"Decision: {action} | Reason: {reason} | Δθ≈{theta_deg:.3f}°", fontsize=12)
        else:
            # Original exposure-focused layout
            under_b = np.clip((0.4 - l_before) / 0.4, 0, 1)
            over_b = np.clip((l_before - 0.9) / 0.1, 0, 1)
            heat_before = np.stack([over_b, np.zeros_like(over_b), under_b], axis=-1)

            under_a = np.clip((0.4 - l_after) / 0.4, 0, 1)
            over_a = np.clip((l_after - 0.9) / 0.1, 0, 1)
            heat_after = np.stack([over_a, np.zeros_like(over_a), under_a], axis=-1)

            gs = fig.add_gridspec(2, 3, height_ratios=[1, 1])

            ax1 = fig.add_subplot(gs[0, 0])
            ax1.imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            ax1.set_title("Original")
            ax1.axis("off")

            ax2 = fig.add_subplot(gs[0, 1])
            ax2.hist((l_before * 255.0).flatten(), bins=64, range=(0, 255), color="gray")
            ax2.set_title("Histogram (before)")
            ax2.set_xlim(0, 255)
            ax2.set_ylabel("Count")

            ax3 = fig.add_subplot(gs[0, 2])
            ax3.imshow(heat_before)
            ax3.set_title("Exposure hotspots (before)")
            ax3.axis("off")

            ax4 = fig.add_subplot(gs[1, 0])
            ax4.imshow(cv2.cvtColor(corrected_bgr, cv2.COLOR_BGR2RGB))
            ax4.set_title("Corrected")
            ax4.axis("off")

            ax5 = fig.add_subplot(gs[1, 1])
            ax5.hist((l_after * 255.0).flatten(), bins=64, range=(0, 255), color="gray")
            ax5.set_title("Histogram (after)")
            ax5.set_xlim(0, 255)
            ax5.set_ylabel("Count")

            ax6 = fig.add_subplot(gs[1, 2])
            ax6.imshow(heat_after)
            ax6.set_title("Exposure hotspots (after)")
            ax6.axis("off")

            fig.suptitle(f"Decision: {action} | Reason: {reason}", fontsize=12)

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        # Save compressed JPEG instead of heavy PNG
        jpg_path = save_path.replace(".png", ".jpg")
        fig.savefig(jpg_path, dpi=100, format="jpg", quality=70, optimize=True)

        plt.close(fig)

        if csv_path is not None:
            self._append_csv(
                csv_path, row_id or pathlib.Path(save_path).stem,
                m_before, m_after, action, reason, ego_info
            )