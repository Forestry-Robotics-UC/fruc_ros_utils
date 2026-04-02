#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# License: MIT (open source)
#
# Description:
#   Vision-related metrics: sharpness, exposure, and action decisions.

"""Image-quality metrics and thresholding helpers."""

import cv2
import numpy as np
from typing import Dict, List, Tuple


def sharpness_score(image: np.ndarray, method: str = "tenengrad") -> float:
    """
    Compute a sharpness score for an image.

    Args:
        image (np.ndarray): Input BGR image.
        method (str): Sharpness metric: 'tenengrad', 'laplacian', 'fft'.

    Returns:
        float: Sharpness score (higher = sharper).
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if method == "laplacian":
        return cv2.Laplacian(gray, cv2.CV_64F).var()

    elif method == "tenengrad":
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        return np.mean(sobelx**2 + sobely**2)

    elif method == "fft":
        f = np.fft.fft2(gray)
        fshift = np.fft.fftshift(f)
        magnitude = np.abs(fshift)
        h, w = magnitude.shape
        crow, ccol = h // 2, w // 2
        mask = np.ones_like(magnitude, dtype=bool)
        r = min(h, w) // 8  # low freq radius
        mask[crow - r:crow + r, ccol - r:ccol + r] = False
        return magnitude[mask].mean()

    else:
        raise ValueError(f"Unknown sharpness method: {method}")

def auto_threshold(scores, percentile: float = 80.0) -> float:
    """
    Compute adaptive threshold based on score distribution.

    Args:
        scores (list[float]): List of sharpness scores.
        percentile (float): Percentile cutoff (0-100).

    Returns:
        float: Threshold value.
    """
    import numpy as np
    if not scores:
        return 0.0
    return float(np.percentile(scores, percentile))



# ---------------- Exposure Metrics ----------------

def exposure_metrics_robust(luma_u8: np.ndarray) -> Dict[str, float]:
    """Compute robust exposure metrics (median/mean/std/dyn_range)."""
    flat = luma_u8.reshape(-1)
    low, high = np.percentile(flat, [0.5, 99.5])
    clipped = flat[(flat >= low) & (flat <= high)]
    if clipped.size == 0:
        clipped = flat

    return {
        "median_clipped": float(np.median(clipped)),
        "mean_clipped": float(np.mean(clipped)),
        "std_clipped": float(np.std(clipped)),
        "p1": float(np.percentile(flat, 1)),
        "p99": float(np.percentile(flat, 99)),
        "dyn_range": float(np.percentile(flat, 99) - np.percentile(flat, 1)),
        "pct_dark": float((flat <= 20).sum()) * 100.0 / flat.size,
        "pct_bright": float((flat >= 235).sum()) * 100.0 / flat.size,
    }


# ---------------- Sharpness Metrics ----------------

def sharpness_metrics(img_bgr) -> Dict[str, float]:
    """Compute Laplacian variance and Tenengrad sharpness metrics."""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    tenengrad = np.mean(sobelx ** 2 + sobely ** 2)
    return {"lap_var": float(lap_var), "tenengrad": float(tenengrad)}


def sharpness_weight(gray: np.ndarray, metric: str = "laplacian", sigma: float = 1.0) -> np.ndarray:
    """Compute per-pixel sharpness weight map (for multi-frame fusion)."""
    g = gray.astype(np.float32) / 255.0
    if metric == "sobel":
        sx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
        sy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
        m = cv2.magnitude(sx, sy)
    else:
        m = cv2.Laplacian(g, cv2.CV_32F, ksize=3)
        m = np.abs(m)
    return cv2.GaussianBlur(m, (0, 0), sigma) + 1e-6
