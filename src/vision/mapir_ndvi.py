#!/usr/bin/env python3
"""Shared helpers for MAPIR NDVI derivation and colorization."""

from __future__ import annotations

from typing import List, Sequence, Tuple

import cv2
import numpy as np


def resolve_channel_defaults(filter_set: str) -> Tuple[int, int, str]:
    """Return default (nir_channel, visible_channel, visible_band_name)."""
    name = str(filter_set).strip().upper()
    if name == "OCN":
        return 2, 0, "orange"
    if name == "RGN":
        return 2, 1, "red"
    if name == "RGB":
        return 2, 2, "red"
    if name == "NGB":
        return 1, 2, "red"
    return 2, 0, "orange"


def resolve_channels(
    *,
    filter_set: str,
    nir_channel: int = -1,
    visible_channel: int = -1,
) -> Tuple[int, int, str]:
    """Resolve channel overrides against the chosen filter-set defaults."""
    nir_default, visible_default, visible_name = resolve_channel_defaults(filter_set)
    if int(nir_channel) < 0:
        nir_channel = nir_default
    if int(visible_channel) < 0:
        visible_channel = visible_default
    nir_channel = int(nir_channel)
    visible_channel = int(visible_channel)
    if nir_channel not in (0, 1, 2):
        raise ValueError("nir_channel must be one of {0,1,2} or negative to use the preset")
    if visible_channel not in (0, 1, 2):
        raise ValueError("visible_channel must be one of {0,1,2} or negative to use the preset")
    return nir_channel, visible_channel, visible_name


def compute_ndvi_from_bgr(
    bgr: np.ndarray,
    *,
    nir_channel: int,
    visible_channel: int,
    eps: float = 1.0e-6,
) -> np.ndarray:
    """Compute NDVI-like index from a 3-channel BGR image."""
    image = np.asarray(bgr, dtype=np.float32)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected a 3-channel image, got shape {image.shape}")
    if eps <= 0.0:
        raise ValueError("eps must be > 0")
    nir = image[:, :, int(nir_channel)]
    visible = image[:, :, int(visible_channel)]
    denom = np.maximum(nir + visible, float(eps))
    ndvi = (nir - visible) / denom
    return np.clip(ndvi, -1.0, 1.0).astype(np.float32, copy=False)


def colorize_ndvi(
    ndvi: np.ndarray,
    *,
    colormap: str = "plant_health",
    colorize_min: float = -1.0,
    colorize_max: float = 1.0,
    custom_colormap: str = "",
) -> np.ndarray:
    """Colorize an NDVI image into BGR8."""
    if not float(colorize_max) > float(colorize_min):
        raise ValueError("colorize_max must be > colorize_min")
    normalized = (np.asarray(ndvi, dtype=np.float32) - float(colorize_min)) / (
        float(colorize_max) - float(colorize_min)
    )
    normalized = np.clip(normalized, 0.0, 1.0).astype(np.float32, copy=False)
    name = str(colormap).strip().lower()
    if name == "gray":
        gray = np.rint(normalized * 255.0).astype(np.uint8, copy=False)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if name == "jet":
        u8 = np.rint(normalized * 255.0).astype(np.uint8, copy=False)
        return cv2.applyColorMap(u8, cv2.COLORMAP_JET)
    if name == "viridis":
        u8 = np.rint(normalized * 255.0).astype(np.uint8, copy=False)
        return cv2.applyColorMap(u8, cv2.COLORMAP_VIRIDIS)
    if name == "custom":
        anchors = parse_custom_colormap(custom_colormap)
        if len(anchors) < 2:
            anchors = _plant_health_anchors()
        return _interpolate_colormap(normalized, anchors)
    return _interpolate_colormap(normalized, _plant_health_anchors())


def parse_custom_colormap(spec: str) -> List[Tuple[float, np.ndarray]]:
    """Parse 'value,r,g,b; value,r,g,b' into sorted BGR anchors."""
    anchors: List[Tuple[float, np.ndarray]] = []
    for item in str(spec).split(";"):
        item = item.strip()
        if not item:
            continue
        parts = [p.strip() for p in item.split(",")]
        if len(parts) != 4:
            continue
        try:
            value = float(parts[0])
            rgb = np.array(
                [float(parts[1]), float(parts[2]), float(parts[3])],
                dtype=np.float32,
            )
        except ValueError:
            continue
        anchors.append((value, np.array([rgb[2], rgb[1], rgb[0]], dtype=np.float32)))
    anchors.sort(key=lambda x: x[0])
    return anchors


def _plant_health_anchors() -> List[Tuple[float, np.ndarray]]:
    # BGR anchors ordered from low NDVI to high NDVI. High values end in green.
    return [
        (0.0, np.array([40.0, 40.0, 165.0], dtype=np.float32)),
        (0.5, np.array([0.0, 220.0, 255.0], dtype=np.float32)),
        (1.0, np.array([0.0, 180.0, 0.0], dtype=np.float32)),
    ]


def _interpolate_colormap(
    normalized: np.ndarray,
    anchors: Sequence[Tuple[float, np.ndarray]],
) -> np.ndarray:
    flat = normalized.reshape(-1)
    values = np.array([a[0] for a in anchors], dtype=np.float32)
    colors = np.stack([a[1] for a in anchors], axis=0).astype(np.float32, copy=False)
    out = np.empty((flat.shape[0], 3), dtype=np.float32)
    for channel in range(3):
        out[:, channel] = np.interp(flat, values, colors[:, channel])
    out = out.reshape(normalized.shape[0], normalized.shape[1], 3)
    return np.clip(np.rint(out), 0.0, 255.0).astype(np.uint8, copy=False)
