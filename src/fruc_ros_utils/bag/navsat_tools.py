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
