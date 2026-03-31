#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# License: MIT (open source)
#
# Description:
#   Generic vision utilities for ROS <-> OpenCV conversion and image enhancement.


import cv2
import numpy as np
from typing import Optional, List, Tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import fruc_ros_utils.utils.metrics.vision as vmetrics

# ---------------- ROS <-> OpenCV helpers ----------------

def is_bayer(encoding: str) -> bool:
    """Check if encoding is a Bayer pattern."""
    return encoding.lower().startswith("bayer_")


def demosaic_bayer_ros(ros_img: Image) -> np.ndarray:
    bridge = CvBridge()
    raw = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    code = bayer_to_code(ros_img.encoding, to_bgr=False)
    bgr = cv2.cvtColor(raw, code)
    return bgr


def demosaic_bayer(cv_img: np.ndarray, encoding: str) -> np.ndarray:
    """Convert a Bayer numpy image (with encoding string) to BGR."""
    return cv2.cvtColor(cv_img, bayer_to_code(encoding, to_bgr=True))


def bayer_to_code(encoding: str, to_bgr: bool = True) -> int:
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


def remosaic_bgr(img_bgr: np.ndarray, pattern: str = "rggb") -> np.ndarray:
    """Convert a BGR image back to a Bayer mosaic (uint8)."""
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


def to_rgb(img: np.ndarray) -> np.ndarray:
    if img.ndim == 2:  # grayscale
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img


# ----------------------- Enhancement primitives -----------------------------

def gray_world_white_balance(img_bgr: np.ndarray) -> np.ndarray:
    """Gray-world white balance correction."""
    img = img_bgr.astype(np.float32)
    mean_b, mean_g, mean_r = [float(img[:, :, i].mean()) + 1e-6 for i in range(3)]
    gray = (mean_b + mean_g + mean_r) / 3.0
    gains = (gray / mean_b, gray / mean_g, gray / mean_r)
    out = img * np.array(gains, dtype=np.float32)[None, None, :]
    return np.clip(out, 0, 255).astype(np.uint8)


def gamma_correct(img_bgr: np.ndarray, gamma: float) -> np.ndarray:
    """Apply gamma correction via LUT."""
    g = max(float(gamma), 1e-3)
    lut = np.array([((i / 255.0) ** g) * 255.0 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(img_bgr, lut)


def estimate_gamma_for_target_mean(curr_mean: float, target: float, gmin: float, gmax: float) -> float:
    """Estimate gamma to move median brightness toward a target, clamped."""
    curr = max(curr_mean, 1.0)
    ratio = target / curr
    gamma = ratio ** 0.7
    return float(np.clip(gamma, gmin, gmax))


def apply_clahe_on_l(img_bgr: np.ndarray, clip_limit: float, tiles: int) -> np.ndarray:
    """Apply CLAHE on L channel in LAB space."""
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tiles, tiles))
    l2 = clahe.apply(l)
    lab2 = cv2.merge((l2, a, b))
    return cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)


def retinex_msr(img_bgr: np.ndarray, scales: List[int] = [15, 80, 250]) -> np.ndarray:
    """Multi-Scale Retinex enhancement."""
    img = img_bgr.astype(np.float32) + 1.0
    log_img = np.log(img)
    msr = np.zeros_like(img)
    for scale in scales:
        blur = cv2.GaussianBlur(img, (0, 0), sigmaX=scale, sigmaY=scale)
        msr += (log_img - np.log(blur + 1.0))
    msr /= len(scales)
    msr = cv2.normalize(msr, None, 0, 255, cv2.NORM_MINMAX)
    return msr.astype(np.uint8)


def lime_enhance(img_bgr: np.ndarray) -> np.ndarray:
    """Low-light Image Enhancement (LIME)."""
    img = img_bgr.astype(np.float32) / 255.0
    illum = np.max(img, axis=2)
    illum = cv2.ximgproc.guidedFilter(illum.astype(np.float32), illum.astype(np.float32), radius=15, eps=1e-3)
    illum = np.expand_dims(illum, axis=2)
    enhanced = img / (illum + 1e-6)
    return np.clip(enhanced, 0, 1) * 255.0


def motion_kernel(length_px: float, angle_rad: float, width: float = 1.0) -> np.ndarray:
    """Create a normalized linear motion blur kernel (PSF)."""
    length = max(1, int(round(length_px)))
    size = max(3, length + 2)
    k = np.zeros((size, size), np.float32)
    center = (size - 1) / 2.0
    dx, dy = np.cos(angle_rad), np.sin(angle_rad)

    num = max(length * 4, 8)
    for i in range(num):
        t = (i / (num - 1)) * (length - 1) - (length - 1) / 2.0
        x, y = center + dx * t, center + dy * t
        xi, yi = int(round(x)), int(round(y))
        if 0 <= yi < size and 0 <= xi < size:
            k[yi, xi] += 1.0
    k = cv2.GaussianBlur(k, (0, 0), width)
    return k / (k.sum() + 1e-6)

    
def richardson_lucy(img_gray: np.ndarray, psf: np.ndarray, iters: int) -> np.ndarray:
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

def deblur_multiframe(center_bgr, context_frames, config, logger=None):
    if not context_frames:
        return center_bgr, False

    ref_gray = cv2.cvtColor(center_bgr, cv2.COLOR_BGR2GRAY)
    aligned = [center_bgr]

    # Align neighbors
    for nbr in context_frames:
        nbr_gray = cv2.cvtColor(nbr, cv2.COLOR_BGR2GRAY)
        if config.mf_align == "flow":
            flow = cv2.calcOpticalFlowFarneback(
                nbr_gray, ref_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
            )
            h, w = ref_gray.shape
            grid_x, grid_y = np.meshgrid(np.arange(w), np.arange(h))
            map_x = (grid_x + flow[..., 0]).astype(np.float32)
            map_y = (grid_y + flow[..., 1]).astype(np.float32)
            aligned.append(cv2.remap(nbr, map_x, map_y,
                                     interpolation=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_REFLECT))
        else:
            aligned.append(nbr)

    # Compute sharpness
    sharpness_all = [vmetrics.sharpness_metrics(f) for f in aligned]
    center_sharp = sharpness_all[0]["lap_var"]

    # Keep only sharper frames
    valid = [(f, m) for f, m in zip(aligned, sharpness_all)
             if m["lap_var"] > center_sharp * config.mf_gain_thresh]

    if not valid:
        if logger: logger.debug("Deblur skipped: no sharper neighbors")
        return center_bgr, False

    # Weighted fusion
    weight_maps = [
        vmetrics.sharpness_weight(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY),
                                  config.mf_sharpness_metric, config.mf_sigma)
        for f, _ in valid
    ]
    W = np.stack(weight_maps, axis=-1)
    Wn = W / (np.sum(W, axis=-1, keepdims=True) + 1e-6)

    acc = np.zeros_like(center_bgr, dtype=np.float32)
    for i, (f, _) in enumerate(valid):
        acc += f.astype(np.float32) * Wn[..., i:i+1]
    fused = np.clip(acc, 0, 255).astype(np.uint8)

    # Check if fused is actually sharper & not too different
    fused_sharp = vmetrics.sharpness_metrics(fused)["lap_var"]
    diff = cv2.absdiff(center_bgr, fused).mean()

    if fused_sharp < center_sharp or diff > 25:  # threshold to avoid ghosting
        if logger: logger.debug(
            "Deblur rejected: fused sharper=%.1f vs center=%.1f, diff=%.1f",
            fused_sharp, center_sharp, diff
        )
        return center_bgr, False

    if logger:
        logger.debug("Deblur accepted: center=%.1f → fused=%.1f", center_sharp, fused_sharp)
    return fused, True



def wiener_deconv(img_gray: np.ndarray, psf: np.ndarray, K: float) -> np.ndarray:
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

def deblur_single(
    img_bgr: np.ndarray,
    imu_msg,
    K: Optional[np.ndarray],
    exposure_time: float,
    config,
    logger: Optional[object] = None,
) -> Tuple[np.ndarray, bool]:
    """
    Single-frame deblur using IMU-estimated PSF and RL/Wiener on luminance.
    """
    if imu_msg is None or K is None:
        if logger: logger.debug("Deblur skipped: missing IMU or intrinsics")
        return img_bgr, False

    # Estimate PSF
    wx = getattr(imu_msg.angular_velocity, "x", 0.0)
    wy = getattr(imu_msg.angular_velocity, "y", 0.0)
    thetax = float(wx) * exposure_time
    thetay = float(wy) * exposure_time
    fx, fy = float(K[0, 0]), float(K[1, 1])
    shift_x, shift_y = fx * thetay, fy * thetax
    length_px = np.hypot(shift_x, shift_y)
    if length_px < config.deblur_psf_min_px:
        return img_bgr, False
    length_px = float(np.clip(length_px, config.deblur_psf_min_px, config.deblur_psf_max_px))
    angle = np.arctan2(shift_y, shift_x + 1e-9)
    psf = motion_kernel(length_px, angle, width=0.8)

    if logger:
        logger.debug(f"Deblur: PSF length={length_px:.2f}px, angle={np.degrees(angle):.1f}°")

    # Apply RL or Wiener on luminance
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
    L = lab[:, :, 0].astype(np.float32) / 255.0
    if config.deblur_algorithm == "wiener":
        Ld = wiener_deconv(L, psf, config.deblur_wiener_k)
    else:
        Ld = richardson_lucy(L, psf, config.deblur_iters)
    lab[:, :, 0] = np.clip(Ld * 255.0, 0, 255).astype(np.uint8)
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR), True


# ------ Choose params recommendations based on bags -------------

def recommend_params(df, cfg_tree: dict) -> dict:
    """
    Recommend parameter values from dataset statistics.
    Works on a nested cfg tree:
    {
      "illumination": {...},
      "raw": {...},
      "deblur": {...},
      "egomotion": {...},
      "retinex_lime": {...},
      "runtime": {...},
    }
    """

    recs = {}

    # --- Illumination thresholds ---
    illum = cfg_tree.get("illumination", {})
    recs["illumination"] = {
        "mean_dark": {
            "current": illum.get("mean_dark"),
            "suggested": df["mean_clipped"].quantile(0.05),
            "reason": "5th percentile of mean_clipped (underexposed tail)"
        },
        "mean_bright": {
            "current": illum.get("mean_bright"),
            "suggested": df["mean_clipped"].quantile(0.95),
            "reason": "95th percentile of mean_clipped (overexposed tail)"
        },
        "pct_dark_th": {
            "current": illum.get("pct_dark_th"),
            "suggested": df["pct_dark"].quantile(0.75),
            "reason": "75th percentile of pct_dark (dark pixel fraction)"
        },
        "pct_bright_th": {
            "current": illum.get("pct_bright_th"),
            "suggested": df["pct_bright"].quantile(0.75),
            "reason": "75th percentile of pct_bright (bright pixel fraction)"
        },
        "dyn_range_min": {
            "current": illum.get("dyn_range_min"),
            "suggested": df["dyn_range"].quantile(0.05),
            "reason": "5th percentile of dyn_range (low contrast detection)"
        },
    }

    # --- RAW processing ---
    raw = cfg_tree.get("raw", {})
    recs["raw"] = {
        "raw_enable_clahe": {
            "current": raw.get("raw_enable_clahe"),
            "suggested": False,
            "reason": "Enable if RAW images look flat/low contrast"
        },
        "raw_p_low": {
            "current": raw.get("raw_p_low"),
            "suggested": 1.0,
            "reason": "Lower if dark clipping is frequent"
        },
        "raw_p_high": {
            "current": raw.get("raw_p_high"),
            "suggested": 98.0,
            "reason": "Lower if highlight saturation is frequent"
        },
    }

    # --- Deblur ---
    deblur = cfg_tree.get("deblur", {})
    recs["deblur"] = {
        "mf_min_sharpness": {
            "current": deblur.get("mf_min_sharpness"),
            "suggested": df["lap_var"].quantile(0.25),
            "reason": "25th percentile LapVar (blurry tail)"
        },
        "deblur_mode": {
            "current": deblur.get("deblur_mode"),
            "suggested": deblur.get("deblur_mode", "multiframe"),
            "reason": "Multiframe is more robust for continuous sequences"
        },
        "deblur_algorithm": {
            "current": deblur.get("deblur_algorithm"),
            "suggested": "lucy",
            "reason": "Lucy-Richardson is usually better for motion blur than Wiener"
        },
    }

    # --- Egomotion ---
    ego = cfg_tree.get("egomotion", {})
    recs["egomotion"] = {
        "ego_theta_thresh_rad": {
            "current": ego.get("ego_theta_thresh_rad"),
            "suggested": 0.0025,
            "reason": "Lower if blur persists, higher if false positives occur"
        }
    }

    # --- Retinex / LIME ---
    retinex = cfg_tree.get("retinex_lime", {})
    recs["retinex_lime"] = {
        "retinex_scales": {
            "current": retinex.get("retinex_scales"),
            "suggested": [15, 80, 180],
            "reason": "Reduce largest scale if halos appear in bright areas"
        },
        "lime_guided_radius": {
            "current": retinex.get("lime_guided_radius"),
            "suggested": 10,
            "reason": "Smaller radius = sharper detail, larger = smoother"
        },
        "lime_guided_eps": {
            "current": retinex.get("lime_guided_eps"),
            "suggested": retinex.get("lime_guided_eps", 0.001),
            "reason": "Keep small (0.001–0.01); larger smooths more but loses detail"
        },
    }

    # --- Runtime ---
    runtime = cfg_tree.get("runtime", {})
    recs["runtime"] = {
        "preserve_bayer": {
            "current": runtime.get("preserve_bayer"),
            "suggested": False,
            "reason": "Keep Bayer only if RAW processing pipeline is enabled"
        },
        "force": {
            "current": runtime.get("force"),
            "suggested": False,
            "reason": "Set True only for benchmarking; False for normal runs"
        }
    }

    return recs
