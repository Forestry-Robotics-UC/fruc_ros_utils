#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Refactored utilities for ROS bag processing. Provides helpers for
#   duration calculation, topic manipulation, and integration with
#   illumination and navsat tools.

"""ROS 1 bag processing utilities and CLI entrypoints."""

# ===== Standard Library =====
import argparse
import os
import sys
import pathlib
from typing import Dict, List, Optional

# ===== Custom Utilities =====
from fruc_ros_utils.utils.logging_utils import get_logger
from fruc_ros_utils.utils.tf_utils import load_extrinsics_from_urdf

# ===== Custom NavSat Tools =====
from fruc_ros_utils.bag.navsat_tools import (
    extract_navsat_records as _extract_navsat_records,
    navsat_export as _navsat_export,
    navsat_summary as _navsat_summary,
    navsat_report as _navsat_report,
)

# Module-level logger — handlers and level applied on first RosbagUtils() instantiation.
logger = get_logger("Bagutils")


def _configure_module_logger() -> None:
    """Apply env-var log level / file overrides. Called once from RosbagUtils.__init__."""
    level = os.environ.get("BAGUTILS_LOG_LEVEL", "INFO").upper()
    log_file = os.environ.get("BAGUTILS_LOG_FILE", None)
    import logging
    logger.setLevel(getattr(logging, level, logging.INFO))
    if log_file and not any(
        isinstance(h, logging.FileHandler) and h.baseFilename == log_file
        for h in logger.handlers
    ):
        import logging as _logging
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        fh = _logging.FileHandler(log_file, mode="a")
        fh.setFormatter(_logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s.%(funcName)s: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        logger.addHandler(fh)

try:
    import argcomplete
except Exception:  # pragma: no cover
    argcomplete = None

# --------------------------------------------------------------------------- #
#                                HELPERS                                      #
# --------------------------------------------------------------------------- #
from fruc_ros_utils.bag.ros1_bag_ops import (
    _iter_bags,
    _iter_messages,
    _discover_bags,
    _resolve_out_bag,
    _resolve_remap_out_bag,
    calculate_bag_duration as _calc_duration,
    remove_topic as _remove_topic,
    remap_topics as _remap_topics,
    print_topic_sizes as _print_topic_sizes,
    change_frame_id as _change_frame_id,
    convert_imu_to_enu as _convert_imu_to_enu,
    convert_camera_info as _convert_camera_info,
)
from fruc_ros_utils.bag.image_pipeline import (
    _is_image_datatype,
    _discover_image_topics,
    _run_with_timeout,
    _decode_image_to_bgr,
    _to_secs_nsecs,
    _pick_stamp,
    _sanitize_topic_for_path,
    _load_extrinsics,
    extract_images as _extract_images,
    extract_images_manifest as _extract_images_manifest,
    images_manifest_to_bag as _images_manifest_to_bag,
)
from fruc_ros_utils.bag.metrics_pipeline import (
    analyze_metrics as _analyze_metrics,
    auto_illumination_from_bag as _auto_illumination_from_bag,
)
from fruc_ros_utils.bag.ndvi_pipeline import mapir_ndvi as _mapir_ndvi


# --------------------------- Config Loader ----------------------------------
from fruc_ros_utils.bag.config import make_cfg_tree, load_yaml, load_configs, merge_configs

# --------------------------------------------------------------------------- #
#                           MAIN CLASS                                        #
# --------------------------------------------------------------------------- #

class RosbagUtils:
    """Utility class for processing ROS bag files."""

    def __init__(self):
        _configure_module_logger()

    def calculate_bag_duration(self, in_path: str, total: bool = False) -> Dict[str, float]:
        return _calc_duration(in_path, total)

    def remove_topic(self, in_path: str, out_path: str, topics: List[str]) -> None:
        return _remove_topic(in_path, out_path, topics)


    def remap_topics(self, in_path: str, out_path: Optional[str], remap: Dict[str, str], overwrite: bool = False) -> None:
        return _remap_topics(in_path, out_path, remap, overwrite)


    def print_topic_sizes(self, in_path: str) -> Dict[str, Dict[str, int]]:
        return _print_topic_sizes(in_path)

    def change_frame_id(self, in_path: str, out_path: str, topics: List[str], new_frame_id: str) -> None:
        return _change_frame_id(in_path, out_path, topics, new_frame_id)

    def convert_imu_to_enu(self, in_path: str, out_path: str, topics: List[str]) -> None:
        return _convert_imu_to_enu(in_path, out_path, topics)

    def convert_camera_info(self, in_path: str, out_path: str, topics: Optional[List[str]] = None) -> None:
        return _convert_camera_info(in_path, out_path, topics)

    def mapir_ndvi(
        self,
        in_path: str,
        out_path: str,
        *,
        image_topic: str = "/mapir/image_raw",
        output_topic: str = "/mapir/indices/ndvi",
        output_encoding: str = "32FC1",
        publish_color: bool = True,
        color_topic: str = "/mapir/indices_color/ndvi",
        colormap: str = "plant_health",
        custom_colormap: str = "",
        colorize_min: float = -1.0,
        colorize_max: float = 1.0,
        filter_set: str = "OCN",
        nir_channel: int = -1,
        visible_channel: int = -1,
        visible_band_name: Optional[str] = None,
        eps: float = 1.0e-6,
    ) -> None:
        return _mapir_ndvi(
            in_path, out_path,
            image_topic=image_topic, output_topic=output_topic,
            output_encoding=output_encoding, publish_color=publish_color,
            color_topic=color_topic, colormap=colormap,
            custom_colormap=custom_colormap, colorize_min=colorize_min,
            colorize_max=colorize_max, filter_set=filter_set,
            nir_channel=nir_channel, visible_channel=visible_channel,
            visible_band_name=visible_band_name, eps=eps,
        )

    def extract_navsat_records(self, in_path: str, topics: List[str]) -> Dict[str, List[Dict]]:
        return _extract_navsat_records(in_path, topics)

    def navsat_export(self, in_path: str, out_dir: str, topics: List[str], csv_name: str = "navsat.csv", kml_name: Optional[str] = None) -> None:
        return _navsat_export(in_path, out_dir, topics, csv_name, kml_name)

    def navsat_summary(self, in_path: str, topics: List[str]) -> Dict[str, Dict]:
        return _navsat_summary(in_path, topics)

    def navsat_report(self, in_path: str, topics: List[str], report_dir: Optional[str] = None) -> Dict[str, Dict]:
        return _navsat_report(in_path, topics, report_dir)

    def extract_images(
        self,
        in_path: str,
        topics: List[str],
        with_context: bool = False,
        ctx_size: int = 3,
        topic_discovery_timeout_s: float = 5.0,
    ) -> Dict[str, List]:
        return _extract_images(in_path, topics, with_context, ctx_size, topic_discovery_timeout_s)

    def extract_images_manifest(
        self,
        in_path: str,
        out_dir: str,
        topics: Optional[List[str]] = None,
        manifest_name: str = "image_manifest",
        manifest_format: str = "csv",
        time_source: str = "auto",
        topic_discovery_timeout_s: float = 5.0,
        startup_timeout_s: float = 25.0,
    ) -> Dict[str, int]:
        return _extract_images_manifest(
            in_path, out_dir, topics, manifest_name, manifest_format,
            time_source, topic_discovery_timeout_s, startup_timeout_s,
        )

    def images_manifest_to_bag(
        self,
        manifest_path: str,
        out_bag: str,
        images_root: Optional[str] = None,
        topic_override: Optional[str] = None,
        frame_id_override: Optional[str] = None,
        output_encoding: str = "bgr8",
        write_time: str = "stamp",
        delimiter_mode: str = "auto",
        strict: bool = False,
    ) -> Dict[str, int]:
        return _images_manifest_to_bag(
            manifest_path, out_bag, images_root, topic_override,
            frame_id_override, output_encoding, write_time, delimiter_mode, strict,
        )


    def analyze_metrics(
        self,
        in_path: str,
        topics: List[str],
        out_file: Optional[str] = None,
        cfg: Optional[dict] = None,
        benchmark: bool = False,
    ) -> Dict[str, Dict]:
        return _analyze_metrics(in_path, topics, out_file, cfg, benchmark)

    def auto_illumination_from_bag(self, cfg: dict) -> None:
        return _auto_illumination_from_bag(cfg)

def print_results(results: Dict, header: Optional[str] = None) -> None:
    """
    Nicely print results dictionaries to CLI, one entry per line.
    Handles nested dicts (prints subkeys indented).
    """
    if header:
        print(header)
    if not results:
        print("No results")
        return

    for k, v in results.items():
        if isinstance(v, dict):
            print(f"{k}:")
            for subk, subv in v.items():
                if isinstance(subv, dict):
                    print(f"  {subk}:")
                    for subsubk, subsubv in subv.items():
                        print(f"    {subsubk:20s} {subsubv}")
                else:
                    print(f"  {subk:20s} {subv}")
        else:
            # Print floats nicely aligned
            if isinstance(v, float):
                print(f"{k:40s} {v:.2f} s")
            else:
                print(f"{k:40s} {v}")

# --------------------------- CLI Parser -------------------------------------

def build_parser(enable_shell_completion: bool = True) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS bag utilities (refactored)")
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity"
    )
    parser.add_argument(
        "--log-file", default=None,
        help="Optional path to a log file (will append)"
    )

    # Common arguments shared by all subcommands
    common_cfg = argparse.ArgumentParser(add_help=False)
    common_cfg.add_argument("--user-config", help="User YAML config file")
    common_cfg.add_argument("--dev-config", help="Developer YAML config file (optional)")
    common_cfg.add_argument("--in", dest="in_path", help="Input bag file or folder")
    common_cfg.add_argument("--out", dest="out_path", help="Output file or folder")
    common_cfg.add_argument("--topics", nargs="+", help="ROS topics to process")
    common_cfg.add_argument("--report", help="Directory for reports")

    sub = parser.add_subparsers(dest="cmd", required=True)

    # -------- analyze_metrics --------
    sp = sub.add_parser("analyze_metrics", parents=[common_cfg],
                        help="Scan dataset to compute metric ranges")
    sp.add_argument("--benchmark", action="store_true",
                    help="Test different correction methods(lucy vs wiener, flow vs ego, clahe vs lime) on sample frames")
    sp = sub.add_parser("auto_illumination", parents=[common_cfg],
                        help="Correct illumination in bag(s)")
    sp.add_argument("--save-bag", dest="save_bag", help="Optional output bag path")
    sp.add_argument("-f", "--force", action="store_true", help="save every image to the report")


    # -------- calculate_bag_duration --------
    sp = sub.add_parser("calculate_bag_duration", parents=[common_cfg],
                       help="Calculate duration of bag(s)")
    sp.add_argument("--total", action="store_true", help="Also show summed duration")

    # -------- remove_topic --------
    sub.add_parser("remove_topic", parents=[common_cfg],
                   help="Remove specific topics from a bag")

    # -------- change_frame_id --------
    sp = sub.add_parser("change_frame_id", parents=[common_cfg],
                        help="Change frame_id on topic(s)")
    sp.add_argument("--new-frame-id", required=True, help="New frame_id to assign")

    # -------- remap_topics --------
    sp = sub.add_parser("remap_topics", parents=[common_cfg],
                        help="Rename topics in a ROS 1 bag")
    sp.add_argument("--remap", nargs="+", required=True, metavar="OLD:NEW",
                    help="Topic rename mappings to apply to the output bag")
    sp.add_argument("--overwrite", action="store_true",
                    help="Overwrite the output bag, or remap in place when --out is omitted")

    # -------- print_topic_sizes --------
    sub.add_parser("print_topic_sizes", parents=[common_cfg],
                   help="Print cumulative topic sizes in a bag")

    # -------- convert_imu_to_enu --------
    sub.add_parser("convert_imu_to_enu", parents=[common_cfg],
                   help="Convert IMU from NED to ENU frame")

    # -------- urdf_extrinsics --------
    sp = sub.add_parser("urdf_extrinsics", parents=[common_cfg],
                        help="Extract transform between two URDF links")
    sp.add_argument("--urdf", required=True, help="Path to URDF file")
    sp.add_argument("--from", dest="parent_link", required=True, help="Source link")
    sp.add_argument("--to", dest="child_link", required=True, help="Target link")
    sp.add_argument("--rotation-only", action="store_true", help="Only output 3x3 rotation")
    sp.add_argument("--translation-only", action="store_true", help="Only output 3x1 translation")

    # -------- navsat_export --------
    sp = sub.add_parser("navsat_export", parents=[common_cfg],
                        help="Extract NavSatFix and export to CSV/KML")
    sp.add_argument("--csv-name", default="navsat.csv", help="CSV output filename")
    sp.add_argument("--kml-name", help="Optional KML output filename")

    # -------- navsat_summary --------
    sub.add_parser("navsat_summary", parents=[common_cfg],
                   help="Quick summary of NavSatFix data")

    # -------- navsat_report --------
    sub.add_parser("navsat_report", parents=[common_cfg],
                   help="Detailed NavSatFix report (requires pandas)")

    # -------- extract_metadata --------
    sp = sub.add_parser("extract_metadata", parents=[common_cfg],
                        help="Extract metadata messages to JSONL")
    sp.add_argument("--max-msgs", type=int, default=0,
                    help="Optional limit per bag (0=all)")

    # -------- extract_images_manifest --------
    sp = sub.add_parser("extract_images_manifest", parents=[common_cfg],
                        help="Extract image topics to PNG and export timestamp manifest")
    sp.add_argument("--manifest-name", default="image_manifest",
                    help="Base name for manifest output file(s)")
    sp.add_argument("--manifest-format", default="csv", choices=["csv", "txt", "both"],
                    help="Manifest format to write")
    sp.add_argument("--time-source", default="auto", choices=["auto", "header", "bag"],
                    help="Timestamp source for manifest rows")
    sp.add_argument("--topic-discovery-timeout", type=float, default=5.0,
                    help="Seconds before topic auto-discovery is aborted (<=0 disables timeout)")
    sp.add_argument("--startup-timeout", type=float, default=25.0,
                    help="When >0, start partial sequential scan immediately and warn if no image appears within this many seconds")
    sp.set_defaults(out_path="extracted_images")

    # -------- images_manifest_to_bag --------
    sp = sub.add_parser("images_manifest_to_bag", parents=[common_cfg],
                        help="Rebuild a ROS1 image bag from PNG files and a manifest")
    sp.add_argument("--images-root", default=None,
                    help="Root directory for image_relpath entries (default: manifest folder)")
    sp.add_argument("--frame-id", dest="frame_id", default=None,
                    help="Override frame_id for all output messages")
    sp.add_argument("--output-encoding", default="bgr8", choices=["bgr8", "rgb8", "mono8"],
                    help="sensor_msgs/Image encoding for output messages")
    sp.add_argument("--write-time", default="stamp", choices=["stamp", "bag"],
                    help="Timestamp used for bag write time")
    sp.add_argument("--manifest-delimiter", default="auto", choices=["auto", "comma", "tab", "semicolon"],
                    help="Delimiter used when reading manifest")
    sp.add_argument("--strict", action="store_true",
                    help="Fail fast on malformed rows or missing image files")

    # -------- colorize_labels --------
    sp = sub.add_parser("colorize_labels", parents=[common_cfg],
                        help="Colorize label images from bag(s) using a color map")
    sp.add_argument("--color-map", dest="color_map",
                    help="YAML/JSON string or file with color_map entries")
    sp.add_argument("--interactive", action="store_true",
                    help="Preview frames; click or press 's' to save")
    sp.add_argument("--save-all", action="store_true",
                    help="Save every colorized frame to --out")
    sp.add_argument("--save-both", action="store_true",
                    help="Save both label colors and overlay when saving")
    sp.add_argument("--overlay-topic", dest="overlay_topic",
                    help="Base image topic to blend with labels")
    sp.add_argument("--overlay-alpha", type=float, default=0.5,
                    help="Overlay alpha (0..1) for label blending")
    sp.add_argument("--overlay-max-dt", type=float, default=0.05,
                    help="Max time delta (s) between overlay image and labels (<=0 disables)")
    sp.add_argument("--stride", type=int, default=1,
                    help="Frame stride (e.g., 2=every other frame)")
    sp.add_argument("--max-frames", type=int, default=0,
                    help="Max frames per bag (0=all)")
    sp.add_argument("--resize", type=float, default=1.0,
                    help="Display scale factor (interactive only)")
    sp.set_defaults(out_path="colorized_labels")

    # -------- mapir_ndvi --------
    sp = sub.add_parser("mapir_ndvi", parents=[common_cfg],
                        help="Derive NDVI from /mapir/image_raw and write a bag with only the derived topic(s)")
    sp.add_argument("--image-topic", default="/mapir/image_raw",
                    help="Input MAPIR image topic to read from the source bag")
    sp.add_argument("--output-topic", default="/mapir/indices/ndvi",
                    help="Output topic for the raw NDVI image")
    sp.add_argument("--output-encoding", default="32FC1", choices=["32FC1", "mono8"],
                    help="Encoding for the raw NDVI image topic")
    sp.add_argument("--publish-color", dest="publish_color", action="store_true", default=True,
                    help="Also write a colorized NDVI topic to the output bag")
    sp.add_argument("--no-publish-color", dest="publish_color", action="store_false",
                    help="Disable the colorized NDVI topic in the output bag")
    sp.add_argument("--color-topic", default="/mapir/indices_color/ndvi",
                    help="Output topic for the colorized NDVI image")
    sp.add_argument("--colormap", default="plant_health",
                    choices=["plant_health", "viridis", "jet", "gray", "custom"],
                    help="Colormap for the colorized NDVI topic; plant_health maps higher NDVI to greener colors")
    sp.add_argument("--custom-colormap", default="",
                    help="Custom colormap 'value,r,g,b; value,r,g,b' used when --colormap=custom")
    sp.add_argument("--colorize-min", type=float, default=-1.0,
                    help="Minimum NDVI value mapped to the start of the colormap")
    sp.add_argument("--colorize-max", type=float, default=1.0,
                    help="Maximum NDVI value mapped to the end of the colormap")
    sp.add_argument("--filter-set", default="OCN",
                    help="MAPIR filter set preset used to resolve default NIR and visible channels")
    sp.add_argument("--nir-channel", type=int, default=-1,
                    help="Override NIR channel index (0=B, 1=G, 2=R); negative uses the filter-set default")
    sp.add_argument("--visible-channel", type=int, default=-1,
                    help="Override visible-band channel index (0=B, 1=G, 2=R); negative uses the filter-set default")
    sp.add_argument("--visible-band-name", default=None,
                    help="Optional label for the visible band in logs")
    sp.add_argument("--eps", type=float, default=1.0e-6,
                    help="Small positive value to stabilize NDVI division")
    if enable_shell_completion and argcomplete:
        argcomplete.autocomplete(parser)
    return parser

# --------------------------- Dispatcher -------------------------------------

def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    global logger
    logger = get_logger("Bagutils", level=args.log_level, log_file=args.log_file)

    user_cfg = load_yaml(getattr(args, "user_config", None))
    default_dev_cfg = pathlib.Path(__file__).resolve().parents[3] / "config" / "dev_defaults.yaml"
    dev_cfg = load_yaml(getattr(args, "dev_config", None)) or load_yaml(str(default_dev_cfg))
    cfg = merge_configs(args, user_cfg, dev_cfg)

    bu = RosbagUtils()

    # Dispatcher
    if args.cmd in ["auto_illumination"]:
        bu.auto_illumination_from_bag(cfg)

    elif args.cmd == "calculate_bag_duration":
        results = bu.calculate_bag_duration(cfg["in_path"], total=args.total)
        print_results(results, "Durations:")

    elif args.cmd == "remove_topic":
        bu.remove_topic(cfg["in_path"], cfg["out_path"], cfg["topics"])

    elif args.cmd == "change_frame_id":
        bu.change_frame_id(cfg["in_path"], cfg["out_path"], cfg["topics"], cfg["new_frame_id"])

    elif args.cmd == "remap_topics":
        remap = {}
        for pair in cfg["remap"]:
            if ":" not in pair:
                parser.error(f"--remap entry must be OLD:NEW, got: {pair}")
            old, new = pair.split(":", 1)
            old = old.strip()
            new = new.strip()
            if not old or not new:
                parser.error(f"--remap entry must be OLD:NEW, got: {pair}")
            remap[old] = new
        bu.remap_topics(cfg["in_path"], cfg.get("out_path"), remap, overwrite=bool(cfg.get("overwrite", False)))

    elif args.cmd == "print_topic_sizes":
        results = bu.print_topic_sizes(cfg["in_path"])

    elif args.cmd == "convert_imu_to_enu":
        bu.convert_imu_to_enu(cfg["in_path"], cfg["out_path"], cfg["topics"])

    elif args.cmd == "navsat_export":
        bu.navsat_export(cfg["in_path"], cfg["out_path"], cfg["topics"],
                         csv_name=cfg.get("csv_name", "navsat.csv"),
                         kml_name=cfg.get("kml_name"))

    elif args.cmd == "navsat_summary":
        results = bu.navsat_summary(cfg["in_path"], cfg["topics"])
        logger.info(f"NavSat Summary: {results}")

    elif args.cmd == "navsat_report":
        results = bu.navsat_report(cfg["in_path"], cfg["topics"], cfg["report"])
        # print_results(results, "NavSat Report:")
    elif args.cmd == "extract_metadata":
        bu.extract_metadata(cfg["in_path"], cfg.get("out_path"), cfg.get("topics"), cfg.get("max_msgs", 0))

    elif args.cmd == "extract_images_manifest":
        in_path = cfg.get("in_path")
        out_path = cfg.get("out_path")
        if not in_path:
            parser.error("extract_images_manifest requires --in")
        if not out_path:
            parser.error("extract_images_manifest requires --out")
        results = bu.extract_images_manifest(
            in_path=in_path,
            out_dir=out_path,
            topics=cfg.get("topics"),
            manifest_name=cfg.get("manifest_name", "image_manifest"),
            manifest_format=cfg.get("manifest_format", "csv"),
            time_source=cfg.get("time_source", "auto"),
            topic_discovery_timeout_s=cfg.get("topic_discovery_timeout", 5.0),
            startup_timeout_s=cfg.get("startup_timeout", 25.0),
        )
        print_results(results, "Image Extraction Summary:")

    elif args.cmd == "images_manifest_to_bag":
        in_path = cfg.get("in_path")
        out_path = cfg.get("out_path")
        if not in_path:
            parser.error("images_manifest_to_bag requires --in (manifest path)")
        if not out_path:
            parser.error("images_manifest_to_bag requires --out (output .bag path)")
        topic_override = None
        if cfg.get("topics"):
            topic_override = cfg["topics"][0]
        results = bu.images_manifest_to_bag(
            manifest_path=in_path,
            out_bag=out_path,
            images_root=cfg.get("images_root"),
            topic_override=topic_override,
            frame_id_override=cfg.get("frame_id"),
            output_encoding=cfg.get("output_encoding", "bgr8"),
            write_time=cfg.get("write_time", "stamp"),
            delimiter_mode=cfg.get("manifest_delimiter", "auto"),
            strict=bool(cfg.get("strict", False)),
        )
        print_results(results, "Bag Rebuild Summary:")

    elif args.cmd == "colorize_labels":
        bu.colorize_label_images(
            cfg["in_path"],
            cfg.get("topics"),
            cfg.get("out_path"),
            color_map=cfg.get("color_map"),
            interactive=bool(cfg.get("interactive")),
            save_all=bool(cfg.get("save_all")),
            save_both=bool(cfg.get("save_both")),
            overlay_topic=cfg.get("overlay_topic"),
            overlay_alpha=cfg.get("overlay_alpha", 0.5),
            overlay_max_dt=cfg.get("overlay_max_dt", 0.05),
            stride=cfg.get("stride", 1),
            max_frames=cfg.get("max_frames", 0),
            resize=cfg.get("resize", 1.0),
        )

    elif args.cmd == "mapir_ndvi":
        bu.mapir_ndvi(
            cfg["in_path"],
            cfg["out_path"],
            image_topic=cfg.get("image_topic", "/mapir/image_raw"),
            output_topic=cfg.get("output_topic", "/mapir/indices/ndvi"),
            output_encoding=cfg.get("output_encoding", "32FC1"),
            publish_color=bool(cfg.get("publish_color", True)),
            color_topic=cfg.get("color_topic", "/mapir/indices_color/ndvi"),
            colormap=cfg.get("colormap", "plant_health"),
            custom_colormap=cfg.get("custom_colormap", ""),
            colorize_min=cfg.get("colorize_min", -1.0),
            colorize_max=cfg.get("colorize_max", 1.0),
            filter_set=cfg.get("filter_set", "OCN"),
            nir_channel=cfg.get("nir_channel", -1),
            visible_channel=cfg.get("visible_channel", -1),
            visible_band_name=cfg.get("visible_band_name"),
            eps=cfg.get("eps", 1.0e-6),
        )
    elif args.cmd == "analyze_metrics":
        results = bu.analyze_metrics(
            cfg["in_path"],
            cfg["topics"],
            cfg["report"],
            cfg,
            benchmark=args.benchmark
        )
        logger.info("Metric ranges: %s", results)
    elif args.cmd == "urdf_extrinsics":
        urdf_path = cfg.get("urdf") or cfg.get("urdf_path")
        if not urdf_path:
            parser.error("urdf_extrinsics requires --urdf")
        T = load_extrinsics_from_urdf(urdf_path, cfg["parent_link"], cfg["child_link"])
        import numpy as np
        np.set_printoptions(precision=4, suppress=True)
        if cfg.get("rotation_only"):
            print(f"Rotation {cfg['parent_link']} -> {cfg['child_link']}:\n{T[:3, :3]}")
        elif cfg.get("translation_only"):
            print(f"Translation {cfg['parent_link']} -> {cfg['child_link']}:\n{T[:3, 3]}")
        else:
            print(f"Transform {cfg['parent_link']} -> {cfg['child_link']}:\n{T}")

    else:
        parser.error(f"Unknown command: {args.cmd}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
