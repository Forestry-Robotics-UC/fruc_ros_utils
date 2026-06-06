#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Navigation satellite utilities: coordinate conversions and
#   CSV/KML exporters for NavSatFix-like records.

"""Navigation-satellite coordinate conversion and export helpers."""

import pathlib
import logging
from typing import List, Dict

import numpy as np
from pyproj import Proj, transform


logger = logging.getLogger(__name__)


# --------------------------------------------------------------------------- #
#                         Coordinate Conversions                              #
# --------------------------------------------------------------------------- #

def lla_to_ecef(lat: float, lon: float, alt: float) -> np.ndarray:
    """
    Convert latitude, longitude, altitude (WGS84) to ECEF coordinates.

    Args:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        alt: Altitude in meters.

    Returns:
        np.ndarray: (x, y, z) in meters.
    """
    proj_lla = Proj(proj="latlong", datum="WGS84")
    proj_ecef = Proj(proj="geocent", datum="WGS84")
    x, y, z = transform(proj_lla, proj_ecef, lon, lat, alt, radians=False)
    return np.array([x, y, z], dtype=float)


def ecef_to_lla(x: float, y: float, z: float) -> np.ndarray:
    """
    Convert ECEF coordinates to latitude, longitude, altitude (WGS84).

    Args:
        x, y, z: Coordinates in meters.

    Returns:
        np.ndarray: (lat, lon, alt), where lat/lon in degrees, alt in meters.
    """
    proj_lla = Proj(proj="latlong", datum="WGS84")
    proj_ecef = Proj(proj="geocent", datum="WGS84")
    lon, lat, alt = transform(proj_ecef, proj_lla, x, y, z, radians=False)
    return np.array([lat, lon, alt], dtype=float)


# --------------------------------------------------------------------------- #
#                                Exporters                                    #
# --------------------------------------------------------------------------- #

def export_navsat_to_csv(
    data: List[Dict],
    out_path: str,
    delimiter: str = ",",
    include_header: bool = True,
) -> pathlib.Path:
    """
    Export NavSatFix-style dicts to a CSV file.

    Args:
        data: List of dicts containing NavSat data (lat, lon, alt, time, status, metrics).
        out_path: Output CSV path.
        delimiter: CSV delimiter (default ",").
        include_header: Write header row (default True).

    Returns:
        pathlib.Path: Path to written CSV file.
    """
    out = pathlib.Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    if not data:
        logger.warning("No NavSat records to export → %s", out)
        out.write_text("")  # create empty file
        return out

    try:
        import pandas as pd
    except ImportError:
        logger.error("pandas is required for CSV export")
        raise

    df = pd.DataFrame(data)
    df.to_csv(out, sep=delimiter, index=False, header=include_header)
    logger.info("Exported %d NavSat records → %s", len(df), out)
    return out


def export_navsat_to_kml(
    data: List[Dict],
    out_path: str,
    name: str = "Trajectory",
) -> pathlib.Path:
    """
    Export NavSatFix-style dicts to a simple KML trajectory.

    Args:
        data: List of dicts containing NavSat data (lat, lon, alt).
        out_path: Output KML path.
        name: Placemark name.

    Returns:
        pathlib.Path: Path to written KML file.
    """
    out = pathlib.Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    if not data:
        logger.warning("No NavSat records to export → %s", out)
        out.write_text(
            "<kml><Document><Placemark><LineString><coordinates/></LineString></Placemark></Document></kml>"
        )
        return out

    coords = "\n".join(
        f"{r.get('lon', 0):.6f},{r.get('lat', 0):.6f},{r.get('alt', 0):.2f}"
        for r in data
    )

    kml_str = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <name>{name}</name>
      <LineString>
        <coordinates>
{coords}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
"""
    out.write_text(kml_str)
    logger.info("Exported %d NavSat records to KML → %s", len(data), out)
    return out


# ---------------------------------------------------------------------------
# Bag-level navsat extraction (stateless counterparts of RosbagUtils methods)
# ---------------------------------------------------------------------------

def extract_navsat_records(in_path: str, topics: List[str]) -> Dict[str, List[Dict]]:
    import os
    import rosbag
    from fruc_ros_utils.bag.ros1_bag_ops import _discover_bags, _iter_bags, _iter_messages
    from fruc_ros_utils.utils.metrics.navsat import cov_metrics

    all_records: List[Dict] = []
    bag_files = _discover_bags(in_path)

    for bag_file in _iter_bags(bag_files, desc="Extracting NavSat"):
        found_topics = {t: False for t in topics}
        with rosbag.Bag(bag_file, "r") as bag:
            for topic, msg, t in _iter_messages(
                bag, desc=f"NavSat {os.path.basename(bag_file)}", topics=topics
            ):
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

    return {"Aggregated": all_records}


def navsat_export(
    in_path: str,
    out_dir: str,
    topics: List[str],
    csv_name: str = "navsat.csv",
    kml_name=None,
) -> None:
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    bag_records = extract_navsat_records(in_path, topics)
    for bag_name, records in bag_records.items():
        if csv_name:
            csv_file = out / f"{pathlib.Path(bag_name).stem}_{csv_name}"
            export_navsat_to_csv(records, csv_file)
        if kml_name:
            kml_file = out / f"{pathlib.Path(bag_name).stem}_{kml_name}"
            export_navsat_to_kml(records, kml_file)


def navsat_summary(in_path: str, topics: List[str]) -> Dict[str, Dict]:
    import numpy as np
    summaries: Dict[str, Dict] = {}
    bag_records = extract_navsat_records(in_path, topics)
    for bag_name, records in bag_records.items():
        status_counts: Dict = {}
        r95 = []
        for r in records:
            status_counts[r["status"]] = status_counts.get(r["status"], 0) + 1
            if not np.isnan(r["r95_major"]):
                r95.append(r["r95_major"])
        summaries[bag_name] = {
            "total": len(records),
            "status_counts": status_counts,
            "r95_major_stats": {
                "mean": np.mean(r95) if r95 else None,
                "max": np.max(r95) if r95 else None,
            },
        }
    return summaries


def navsat_report(
    in_path: str, topics: List[str], report_dir=None
) -> Dict[str, Dict]:
    import json
    try:
        import pandas as pd
    except ImportError:
        logger.error("pandas required for navsat_report")
        return {}

    reports: Dict[str, Dict] = {}
    bag_records = extract_navsat_records(in_path, topics)

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
        if report_dir:
            csv_file = out_path / f"{pathlib.Path(bag_name).stem}_navsat_report.csv"
            json_file = out_path / f"{pathlib.Path(bag_name).stem}_navsat_report.json"
            try:
                df.to_csv(csv_file, index=False)
                with open(json_file, "w") as f:
                    json.dump(reports[bag_name], f, indent=2)
                logger.info("Saved NavSat report for %s → %s / %s", bag_name, csv_file, json_file)
            except Exception as e:
                logger.error("Failed to save report for %s: %s", bag_name, e)

    return reports
