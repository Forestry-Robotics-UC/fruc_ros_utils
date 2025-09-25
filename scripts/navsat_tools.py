# navsat_tools.py
from __future__ import annotations

import os
import csv
import logging
from datetime import datetime
from typing import Iterable, List, Dict, Optional
from concurrent.futures import ProcessPoolExecutor, as_completed
import tempfile
import functools
import numpy as np

# Optional deps used for CSV input/output convenience in quality_report()
try:
    import pandas as pd  # type: ignore
except Exception:  # pragma: no cover
    pd = None  # we'll guard at runtime

# ROS deps
import rosbag  # type: ignore
from sensor_msgs.msg import NavSatFix  # type: ignore

# tqdm is optional (nice progress bars)
try:
    from tqdm import tqdm  # type: ignore
except Exception:  # pragma: no cover
    def tqdm(x, **k): return x  # fallback: no progress bars

logger = logging.getLogger(__name__)

# Chi-square for 2 DoF at 95% (used to scale ellipse axes)
CHI2_2_95 = 5.991


# ----------------------------- Helpers -----------------------------

def _decode_status(code: int) -> str:
    """sensor_msgs/NavSatStatus.status"""
    return {-1: "NO_FIX", 0: "FIX", 1: "SBAS_FIX", 2: "GBAS_FIX"}.get(int(code), f"UNKNOWN({code})")


def _decode_service(mask: int) -> str:
    """sensor_msgs/NavSatStatus.service bitmask"""
    parts = []
    if mask & 1: parts.append("GPS")
    if mask & 2: parts.append("GLONASS")
    if mask & 4: parts.append("COMPASS")   # BeiDou in some stacks
    if mask & 8: parts.append("GALILEO")
    return "|".join(parts) if parts else "NONE"


def _decode_cov_type(code: int) -> str:
    """sensor_msgs/NavSatFix.position_covariance_type"""
    return {0: "UNKNOWN", 1: "APPROXIMATED", 2: "DIAGONAL_KNOWN", 3: "KNOWN"}.get(int(code), f"UNKNOWN({code})")


def _cov_metrics(cov: List[float]) -> Dict[str, float]:
    """
    Compute covariance-derived metrics from a 3x3 ENU covariance (m^2), row-major 9 elems.
    Returns 1σ stds, horizontal RMS, and 95% ellipse major/minor + orientation.
    """
    nan = float("nan")
    if not cov or len(cov) != 9:
        return {
            "sigma_e": nan, "sigma_n": nan, "sigma_u": nan, "sigma_h": nan,
            "r95_major": nan, "r95_minor": nan, "ellipse_angle_deg": nan,
        }

    C = np.asarray(cov, dtype=float).reshape(3, 3)
    # Clamp negatives to zero before sqrt to avoid tiny negative numerical noise
    sigma_e = float(np.sqrt(max(C[0, 0], 0.0)))
    sigma_n = float(np.sqrt(max(C[1, 1], 0.0)))
    sigma_u = float(np.sqrt(max(C[2, 2], 0.0)))
    sigma_h = float(np.sqrt(max(C[0, 0] + C[1, 1], 0.0)))

    # Horizontal ellipse from 2x2 block
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


def _stats(vals: List[float]) -> Optional[Dict[str, float]]:
    """Simple stats helper used by quality_summary()."""
    if not vals:
        return None
    a = np.asarray(vals, dtype=float)
    a = a[~np.isnan(a)]
    if a.size == 0:
        return None
    return {
        "mean": float(a.mean()),
        "median": float(np.median(a)),
        "p90": float(np.percentile(a, 90)),
        "max": float(a.max()),
        "n": int(a.size),
    }


def _trim_iqr(x: np.ndarray) -> np.ndarray:
    """
    IQR outlier trimming mask: keep values within [Q1 - 1.5*IQR, Q3 + 1.5*IQR].
    Returns a boolean mask for x[~nan].
    """
    x = x[~np.isnan(x)]
    if x.size == 0:
        return np.zeros(0, dtype=bool)
    q1, q3 = np.percentile(x, [25, 75])
    iqr = q3 - q1
    lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
    return (x >= lo) & (x <= hi)


def _basic_stats(x: np.ndarray, with_minmax: bool = False) -> Dict[str, float]:
    """Mean/std (& optional min/max) with NaN-safety."""
    x = x[~np.isnan(x)]
    if x.size == 0:
        d = {"mean": float("nan"), "std": float("nan"), "n": 0}
        if with_minmax:
            d.update({"min": float("nan"), "max": float("nan")})
        return d
    d = {"mean": float(x.mean()), "std": float(x.std(ddof=1)) if x.size > 1 else 0.0, "n": int(x.size)}
    if with_minmax:
        d.update({"min": float(x.min()), "max": float(x.max())})
    return d


# --------------------------- Main Class ----------------------------

class NavSatExporter:
    """
    Read NavSatFix from ROS .bag files, export CSV, and compute quality reports.

    Public API:
      - export_csv(folder, out_path="navsatfix_export.csv", recursive=True)
      - quality_summary(folder, recursive=True)
      - quality_report(folder=None, csv_path=None, recursive=True)  # either bags OR CSV
    """

    def __init__(self, topic: str = "/fix"):
        self.topic = topic

    # ----------- File enumeration -----------

    def _iter_bag_files(self, folder: str, recursive: bool) -> Iterable[str]:
        if recursive:
            for root, _, files in os.walk(folder):
                for f in files:
                    if f.endswith(".bag"):
                        yield os.path.join(root, f)
        else:
            for f in os.listdir(folder):
                if f.endswith(".bag"):
                    yield os.path.join(folder, f)

        # ----------- CSV export -----------

    # --- add this helper in NavSatExporter class (before export_csv) ---
    def _export_one_bag(self, bag_file: str, tmp_csv_path: str,  show_progress: bool = True) -> int:
        """
        Export one bag to a temporary CSV. Returns number of rows written.
        """
        headers = [
            "bag_file", "ros_time_sec", "header_stamp_sec", "iso_time", "topic",
            "latitude_deg", "longitude_deg", "altitude_m",
            "status_code", "status_label", "service_mask", "service_label",
            "cov_type_code", "cov_type_label",
            "cov_xx", "cov_xy", "cov_xz", "cov_yx", "cov_yy", "cov_yz", "cov_zx", "cov_zy", "cov_zz",
            "sigma_e_m", "sigma_n_m", "sigma_u_m", "sigma_h_m",
            "r95_major_m", "r95_minor_m", "ellipse_angle_deg",
        ]
        # 1 MiB buffered writer to reduce syscall overhead
        with open(tmp_csv_path, "w", newline="", buffering=1024*1024) as f:
            w = csv.writer(f)
            w.writerow(headers)
            write = w.writerow  # local bind for speed
            _decode_status_loc = _decode_status
            _decode_service_loc = _decode_service
            _decode_cov_type_loc = _decode_cov_type
            _cov_metrics_loc = _cov_metrics

            try:
                # allow_unindexed=True avoids index rebuilds (slow) on some bags
                with rosbag.Bag(bag_file, "r", allow_unindexed=True) as bag:
                    iterator = bag.read_messages(topics=[self.topic])
                    if show_progress:
                            # We avoid bag.get_message_count(...) to prevent index rebuilds; no total => spinner + count
                            iterator = tqdm(iterator, desc=f"{os.path.basename(bag_file)}", unit="msg")

                    for topic_name, msg, t in iterator:
                        # Fast path: ensure we're on NavSatFix
                        if "NavSatFix" not in msg._type:
                            continue

                        status_code = int(getattr(msg.status, "status", -999))
                        service_mask = int(getattr(msg.status, "service", 0))
                        cov = list(getattr(msg, "position_covariance", [float("nan")] * 9))
                        met = _cov_metrics_loc(cov)

                        # timestamps
                        try:
                            stamp_sec = float(msg.header.stamp.to_sec())
                        except Exception:
                            stamp_sec = float("nan")
                        ros_time = float(t.to_sec())
                        # avoid heavy datetime on hot path when possible
                        if stamp_sec == stamp_sec:
                            iso = datetime.utcfromtimestamp(stamp_sec).isoformat() + "Z"
                            header_stamp = f"{stamp_sec:.6f}"
                        else:
                            iso = datetime.utcfromtimestamp(ros_time).isoformat() + "Z"
                            header_stamp = ""

                        # pre-format a few floats only; leave many as raw to let csv handle strings
                        row = [
                            os.path.basename(bag_file),
                            f"{ros_time:.6f}",
                            header_stamp,
                            iso,
                            topic_name,
                            f"{msg.latitude:.9f}",
                            f"{msg.longitude:.9f}",
                            f"{msg.altitude:.3f}",
                            status_code, _decode_status_loc(status_code),
                            service_mask, _decode_service_loc(service_mask),
                            int(getattr(msg, "position_covariance_type", 0)),
                            _decode_cov_type_loc(int(getattr(msg, "position_covariance_type", 0))),
                            *[f"{c:.6f}" if c == c else "" for c in cov],
                            f"{met['sigma_e']:.3f}" if met['sigma_e'] == met['sigma_e'] else "",
                            f"{met['sigma_n']:.3f}" if met['sigma_n'] == met['sigma_n'] else "",
                            f"{met['sigma_u']:.3f}" if met['sigma_u'] == met['sigma_u'] else "",
                            f"{met['sigma_h']:.3f}" if met['sigma_h'] == met['sigma_h'] else "",
                            f"{met['r95_major']:.3f}" if met['r95_major'] == met['r95_major'] else "",
                            f"{met['r95_minor']:.3f}" if met['r95_minor'] == met['r95_minor'] else "",
                            f"{met['ellipse_angle_deg']:.2f}" if met['ellipse_angle_deg'] == met['ellipse_angle_deg'] else "",
                        ]
                        write(row)
            except Exception as e:
                logger.error(f"[export_csv one] {bag_file}: {e}", exc_info=True)
                return 0
        return sum(1 for _ in open(tmp_csv_path, "r", encoding="utf-8", errors="ignore")) - 1  # minus header

    # --- replace export_csv() with the parallel driver ---
    def export_csv(self, folder: str, out_path: str = "navsatfix_export.csv", recursive: bool = True, workers: int = max(os.cpu_count() or 2, 2)) -> None:
        bag_files = list(self._iter_bag_files(folder, recursive))
        if not bag_files:
            logger.warning(f"No .bag files found in {folder}")
            return

        logger.info(f"Exporting {len(bag_files)} bag(s) on topic '{self.topic}' using {workers} worker(s)")
        tmp_dir = tempfile.TemporaryDirectory()

        total_rows = 0

        if workers == 1:
            # --- SERIAL PATH (nice per-message tqdm per bag) ---
            for i, bag_file in enumerate(bag_files):
                tmp_csv = os.path.join(tmp_dir.name, f"part_{i:05d}.csv")
                total_rows += max(0, NavSatExporter(self.topic)._export_one_bag(bag_file, tmp_csv, show_progress=True))
        else:
            # --- PARALLEL PATH (clean global tqdm over bags) ---
            futures = []
            with ProcessPoolExecutor(max_workers=workers) as ex:
                for i, bag_file in enumerate(bag_files):
                    tmp_csv = os.path.join(tmp_dir.name, f"part_{i:05d}.csv")
                    futures.append(ex.submit(NavSatExporter(self.topic)._export_one_bag, bag_file, tmp_csv, False))

                for fut in tqdm(as_completed(futures), total=len(futures), desc="Bags exported", unit="bag"):
                    total_rows += max(0, fut.result())

        # Concatenate parts with a single header
        header_written = False
        with open(out_path, "w", newline="") as out_f:
            for i in range(len(bag_files)):
                part = os.path.join(tmp_dir.name, f"part_{i:05d}.csv")
                try:
                    with open(part, "r", newline="") as pf:
                        for j, line in enumerate(pf):
                            if j == 0 and header_written:
                                continue
                            out_f.write(line)
                    header_written = True
                except FileNotFoundError:
                    continue

        tmp_dir.cleanup()
        logger.info(f"NavSatFix CSV written: {out_path} (rows: {total_rows})")


        # ----------- Simple quality summary (bags only) -----------

    def quality_summary(self, folder: str, recursive: bool = True) -> Dict[str, object]:
        bag_files = list(self._iter_bag_files(folder, recursive))
        counts = {-1: 0, 0: 0, 1: 0, 2: 0}
        r95, sig_h, total = [], [], 0

        for bag_file in tqdm(bag_files, desc=f"Scanning {self.topic}", unit="bag"):
            try:
                with rosbag.Bag(bag_file, "r") as bag:
                    for _, msg, _ in bag.read_messages(topics=[self.topic]):
                        if not 'NavSatFix' in msg._type:
                            continue
                        total += 1
                        s = int(getattr(msg.status, "status", -999))
                        if s in counts:
                            counts[s] += 1
                        met = _cov_metrics(list(getattr(msg, "position_covariance", [float("nan")] * 9)))
                        if met["r95_major"] == met["r95_major"]:
                            r95.append(met["r95_major"])
                        if met["sigma_h"] == met["sigma_h"]:
                            sig_h.append(met["sigma_h"])
            except Exception as e:
                logger.error(f"[quality_summary] {bag_file}: {e}", exc_info=True)

        return {
            "topic": self.topic,
            "total_msgs": total,
            "status_counts": counts,
            "status_labels": {k: _decode_status(k) for k in counts},
            "r95_major_stats": _stats(r95),
            "sigma_h_stats": _stats(sig_h),
        }
        # ----------- Rich quality report (bags OR CSV) -----------

    def quality_report(
        self,
        folder: str | None = None,
        csv_path: str | None = None,
        recursive: bool = True,
    ) -> Dict[str, object]:
        """
        Build a GPS quality report from either:
          - a CSV exported by export_csv(), OR
          - directly from .bag files.

        Returns a dict with:
          "topic", "source", "total_msgs",
          "status_percent" {code: pct}, "status_labels" {code: label},
          "covariance_raw"/"covariance_norm" (per cov element: mean, std, n, and min/max for raw),
          "metrics_raw"/"metrics_norm" (sigma_e/n/u/h, r95_major/minor, ellipse_angle_deg).
        """
        if (csv_path is None) == (folder is None):
            raise ValueError("Provide either csv_path OR folder (but not both).")

        cov_names = ["cov_xx", "cov_xy", "cov_xz", "cov_yx", "cov_yy", "cov_yz", "cov_zx", "cov_zy", "cov_zz"]
        metric_names = ["sigma_e_m", "sigma_n_m", "sigma_u_m", "sigma_h_m", "r95_major_m", "r95_minor_m", "ellipse_angle_deg"]

        # ----- Load dataframe -----
        if csv_path is not None:
            if pd is None:
                raise RuntimeError("pandas is required to read CSV; install pandas or use folder=bags mode.")
            df = pd.read_csv(csv_path, low_memory=False)
            # tolerate alternate metric column names
            rename_map = {
                "sigma_e": "sigma_e_m", "sigma_n": "sigma_n_m", "sigma_u": "sigma_u_m", "sigma_h": "sigma_h_m",
                "r95_major": "r95_major_m", "r95_minor": "r95_minor_m",
            }
            df = df.rename(columns=rename_map)
            source = "csv"
        else:
            # Build a minimal dataframe straight from bags
            rows = []
            for bag_file in (self._iter_bag_files(folder or "", recursive)):
                try:
                    with rosbag.Bag(bag_file, "r") as bag:
                        for _, msg, _ in bag.read_messages(topics=[self.topic]):
                            if not "NavSatFix" in msg._type:
                                    continue
                            status_code = int(getattr(msg.status, "status", -999))
                            cov = list(getattr(msg, "position_covariance", [float("nan")] * 9))
                            met = _cov_metrics(cov)
                            rows.append({
                                "status_code": status_code,
                                "cov_xx": cov[0], "cov_xy": cov[1], "cov_xz": cov[2],
                                "cov_yx": cov[3], "cov_yy": cov[4], "cov_yz": cov[5],
                                "cov_zx": cov[6], "cov_zy": cov[7], "cov_zz": cov[8],
                                "sigma_e_m": met["sigma_e"], "sigma_n_m": met["sigma_n"],
                                "sigma_u_m": met["sigma_u"], "sigma_h_m": met["sigma_h"],
                                "r95_major_m": met["r95_major"], "r95_minor_m": met["r95_minor"],
                                "ellipse_angle_deg": met["ellipse_angle_deg"],
                            })
                except Exception as e:
                    logger.error(f"[quality_report] {bag_file}: {e}", exc_info=True)
            if pd is None:
                # minimal dependency fallback: return raw lists if no pandas at all
                import json
                total = len(rows)
                status_counts: Dict[int, int] = {}
                for r in rows:
                    status_counts[r["status_code"]] = status_counts.get(r["status_code"], 0) + 1
                status_percent = {int(k): 100.0 * v / total for k, v in status_counts.items()} if total else {}
                return {
                    "topic": self.topic,
                    "source": "bags",
                    "total_msgs": total,
                    "status_percent": status_percent,
                    "status_labels": {k: _decode_status(k) for k in status_percent},
                    "covariance_raw": {}, "covariance_norm": {},
                    "metrics_raw": {}, "metrics_norm": {},
                    "note": "Install pandas to compute full stats in quality_report().",
                }
            df = pd.DataFrame.from_records(rows)
            source = "bags"

        if df.empty:
            return {
                "topic": self.topic, "source": source, "total_msgs": 0,
                "status_percent": {}, "status_labels": {},
                "covariance_raw": {}, "covariance_norm": {},
                "metrics_raw": {}, "metrics_norm": {},
            }

        # ----- Status distribution (% by code) -----
        total = int(len(df))
        status_counts = df["status_code"].value_counts(dropna=False).to_dict() if "status_code" in df.columns else {}
        status_percent = {int(k): (100.0 * float(v) / total) for k, v in status_counts.items()} if total else {}
        status_labels = {int(k): _decode_status(int(k)) for k in status_percent.keys()}

        # ----- Stats helpers on dataframe -----
        def series_to_stats(name: str, with_minmax: bool) -> Dict[str, float]:
            if name not in df.columns:
                return _basic_stats(np.array([]), with_minmax=with_minmax)
            return _basic_stats(df[name].to_numpy(dtype=float), with_minmax=with_minmax)

        def series_to_trimmed_stats(name: str) -> Dict[str, float]:
            if name not in df.columns:
                return _basic_stats(np.array([]), with_minmax=False)
            x = df[name].to_numpy(dtype=float)
            x = x[~np.isnan(x)]
            if x.size == 0:
                return _basic_stats(x, with_minmax=False)
            mask = _trim_iqr(x)
            x_trim = x[mask]
            return _basic_stats(x_trim, with_minmax=False)

        cov_raw = {k: series_to_stats(k, with_minmax=True) for k in cov_names}
        cov_norm = {k: series_to_trimmed_stats(k) for k in cov_names}
        met_raw = {k: series_to_stats(k, with_minmax=True) for k in metric_names}
        met_norm = {k: series_to_trimmed_stats(k) for k in metric_names}

        return {
            "topic": self.topic,
            "source": source,
            "total_msgs": total,
            "status_percent": status_percent,
            "status_labels": status_labels,
            "covariance_raw": cov_raw,
            "covariance_norm": cov_norm,
            "metrics_raw": met_raw,
            "metrics_norm": met_norm,
        }
