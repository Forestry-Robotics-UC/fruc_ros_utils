#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
#
# Description:
#   GPS / NavSat quality metrics for use with ROS bagutils and
#   exporters. Provides covariance-based accuracy estimates
#   and ellipse parameters.

import numpy as np
from typing import List, Dict

# Chi-square quantile for 95% confidence (2 DoF)
CHI2_2_95 = 5.991


def cov_metrics(cov: List[float]) -> Dict[str, float]:
    """
    Compute covariance-derived metrics from a 3x3 ENU covariance (m^2).

    Args:
        cov (list): Row-major 9-element covariance matrix.

    Returns:
        dict: {
          "sigma_e", "sigma_n", "sigma_u", "sigma_h",
          "r95_major", "r95_minor", "ellipse_angle_deg"
        }
    """
    nan = float("nan")
    if not cov or len(cov) != 9:
        return {
            "sigma_e": nan, "sigma_n": nan, "sigma_u": nan, "sigma_h": nan,
            "r95_major": nan, "r95_minor": nan, "ellipse_angle_deg": nan,
        }

    C = np.asarray(cov, dtype=float).reshape(3, 3)

    # Standard deviations
    sigma_e = float(np.sqrt(max(C[0, 0], 0.0)))
    sigma_n = float(np.sqrt(max(C[1, 1], 0.0)))
    sigma_u = float(np.sqrt(max(C[2, 2], 0.0)))
    sigma_h = float(np.sqrt(max(C[0, 0] + C[1, 1], 0.0)))

    # Horizontal ellipse (95%)
    Ch = C[:2, :2]
    vals, vecs = np.linalg.eig(Ch)
    vals = np.maximum(vals, 0.0)  # no negative variances
    r95 = np.sqrt(vals * CHI2_2_95)
    major_idx = int(np.argmax(vals))
    angle_deg = float(np.degrees(np.arctan2(vecs[1, major_idx], vecs[0, major_idx])))

    return {
        "sigma_e": sigma_e,
        "sigma_n": sigma_n,
        "sigma_u": sigma_u,
        "sigma_h": sigma_h,
        "r95_major": float(np.max(r95)),
        "r95_minor": float(np.min(r95)),
        "ellipse_angle_deg": angle_deg,
    }
