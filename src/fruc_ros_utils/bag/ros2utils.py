#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT
#
# Description:
#   Multi-action utilities for ROS 2 bags (.db3).  Mirrors the ROS 1 bagutils
#   structure and uses the same logging system from fruc_ros_utils.utils.logging_utils.
#
#   Provides: topic listing, duration computation, generic JSON extraction.
#

"""ROS 2 bag inspection and ROS 2 -> ROS 1 conversion utilities."""

import argparse
import os
import sys
import json
import traceback
import sqlite3
import importlib
import inspect
import logging
import subprocess
import tempfile
from collections import defaultdict
import re
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
import yaml

try:
    import argcomplete
except Exception:
    argcomplete = None

# ===== ROS 2 =====
try:
    import rosbag2_py
    HAS_ROSBAG2 = True
except Exception:
    rosbag2_py = None
    HAS_ROSBAG2 = False
import rclpy.serialization
import numpy as np
from sensor_msgs.msg import PointCloud2

from fruc_ros_utils.bag import ros2_inspector as _insp
from fruc_ros_utils.bag import ros2_topic_filter as _tfilter
from fruc_ros_utils.bag import ros2_validation as _valid
from fruc_ros_utils.bag import ouster_conv_helpers as _ouster
from fruc_ros_utils.bag import ros2_converter as _conv

from fruc_ros_utils.bag._ros2_helpers import (
    _ROS2_MSG_TYPE_SEPARATOR,
    _SIZE_SUFFIXES,
    _DURATION_SUFFIXES,
    _FAILED_PARSE_MSGTYPE_RE,
    _OPTIONAL_DROPPED_ROS2_TYPES,
    _FFMPEG_PACKET_ROS2_TYPES,
    _STANDARD_ROS_TYPE_PREFIXES,
    _make_progress_bar,
    _debug_path_state,
    _debug_topic_count_summary,
    _is_standard_ros_msg_type,
    _is_ffmpeg_packet_msg_type,
    _safe_rosbag2_metadata_scalar,
    _merge_topic_lists,
    _parse_threshold_value,
    _parse_size_bytes,
    _parse_duration_seconds,
    _format_bytes,
    _natural_sort_key,
)

# ===== Internal logging (shared with ROS 1 utils) =====
from fruc_ros_utils.utils.logging_utils import get_logger

# --------------------------------------------------------------------------- #
#                           LOGGER INITIALIZATION                             #
# --------------------------------------------------------------------------- #
_default_level = "INFO"
_default_file  = None
logger = get_logger("Ros2utils", level=_default_level, log_file=_default_file)

# convenience aliases (consistent with bagutils)
log, warn, err = (
    logger.info,
    logger.warning,
    logger.error,
)


# --------------------------------------------------------------------------- #
#                              CORE CLASS                                     #
# --------------------------------------------------------------------------- #
class Ros2BagUtils:
    """Utility class for processing ROS 2 bags (rosbag2 SQLite3)."""

    # ---------- internal helpers ----------
    def _open_reader(self, path: str):
        return _insp.open_ros2_reader(path)

    @staticmethod
    def _ensure_ros1_tf2_typestore(dst_store) -> None:
        return _insp._ensure_ros1_tf2_typestore(dst_store)

    @staticmethod
    def _cleanup_partial_output(dst_path: Union[str, Path], context: str = "") -> None:
        return _insp._cleanup_partial_output(dst_path, context)

    @staticmethod
    def _debug_log_topic_descriptions(context: str, topics: List[object], limit: int = 20) -> None:
        return _insp._debug_log_topic_descriptions(context, topics, limit)

    def _debug_log_opened_ros2_reader(self, reader, path: Union[str, Path], context: str) -> None:
        return _insp._debug_log_opened_ros2_reader(reader, path, context)

    def _debug_log_ros2_bag_summary(self, bag_path: Union[str, Path], context: str) -> None:
        return _insp._debug_log_ros2_bag_summary(bag_path, context)

    @staticmethod
    def _debug_log_subprocess_result(context: str, cmd: List[str], result: subprocess.CompletedProcess) -> None:
        return _insp._debug_log_subprocess_result(context, cmd, result)

    @staticmethod
    def _resolve_rosbags_typestore(typestore: object):
        return _insp._resolve_rosbags_typestore(typestore)

    # ---------- actions ----------
    def list_topics(self, bag_path: str) -> Dict[str, str]:
        return _insp.list_topics(bag_path)

    @staticmethod
    def _bag_size_bytes_raw(bag_path: Union[str, Path]) -> int:
        return _insp.bag_size_bytes_raw(bag_path)

    def _ros2_bag_info_metadata(self, bag_path: str) -> Dict[str, object]:
        return _insp._ros2_bag_info_metadata(bag_path)

    def info(self, bag_path: str) -> Dict[str, object]:
        return _insp.info(bag_path)

    def bag_duration(self, bag_path: str) -> float:
        return _insp.bag_duration(bag_path)

    def bag_size_bytes(self, bag_path: str) -> int:
        return _insp.bag_size_bytes(bag_path)

    def _ensure_output_available(self, dst_path: str, overwrite: bool) -> None:
        return _insp.ensure_output_available(dst_path, overwrite)

    # ---------- ROS2 → ROS1 conversion ----------
    def convert_to_ros1(
        self,
        bag_path: str,
        out_path: Optional[str] = None,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        validate: bool = True,
        preserve_lidar_fields: bool = False,
        lidar_topic: Optional[str] = None,
        overwrite: bool = False,
        split_duration: Optional[str] = None,
        split_size: Optional[str] = None,
        decode_ouster: bool = False,
        decode_ffmpeg: bool = False,
        output_mode: str = "points",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        keep_raw_ouster: bool = False,
        metadata_file: Optional[str] = None,
    ) -> Union[str, List[str]]:
        split_duration_s = _parse_duration_seconds(split_duration)
        split_size_bytes = _parse_size_bytes(split_size)
        if split_duration_s or split_size_bytes:
            return self._convert_to_ros1_with_split(
                bag_path=bag_path,
                out_path=out_path,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
                remap=remap,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                validate=validate,
                preserve_lidar_fields=preserve_lidar_fields,
                lidar_topic=lidar_topic,
                overwrite=overwrite,
                split_duration_s=split_duration_s,
                split_size_bytes=split_size_bytes,
                decode_ouster=decode_ouster,
                decode_ffmpeg=decode_ffmpeg,
                output_mode=output_mode,
                points_topic=points_topic,
                depth_topic=depth_topic,
                imu_topic=imu_topic,
                keep_raw_ouster=keep_raw_ouster,
                metadata_file=metadata_file,
            )
        src = Path(bag_path).resolve()
        if (
            src.is_dir()
            and not split_duration_s
            and not split_size_bytes
            and self._existing_split_member_count(str(src)) > 1
        ):
            # Preserve an existing rosbag2 split layout by converting each member file
            # into a matching ROS1 chunk instead of concatenating the directory into one bag.
            return self._convert_existing_split_members_to_ros1(
                bag_path=bag_path,
                out_path=out_path,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
                remap=remap,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                validate=validate,
                preserve_lidar_fields=preserve_lidar_fields,
                lidar_topic=lidar_topic,
                overwrite=overwrite,
                decode_ouster=decode_ouster,
                decode_ffmpeg=decode_ffmpeg,
                output_mode=output_mode,
                points_topic=points_topic,
                depth_topic=depth_topic,
                imu_topic=imu_topic,
                keep_raw_ouster=keep_raw_ouster,
                metadata_file=metadata_file,
            )
        return self._convert_single_to_ros1(
            bag_path=bag_path,
            out_path=out_path,
            include_topics=include_topics,
            exclude_topics=exclude_topics,
            remap=remap,
            src_typestore=src_typestore,
            dst_typestore=dst_typestore,
            validate=validate,
            preserve_lidar_fields=preserve_lidar_fields,
            lidar_topic=lidar_topic,
            overwrite=overwrite,
            decode_ouster=decode_ouster,
            decode_ffmpeg=decode_ffmpeg,
            output_mode=output_mode,
            points_topic=points_topic,
            depth_topic=depth_topic,
            imu_topic=imu_topic,
            keep_raw_ouster=keep_raw_ouster,
            metadata_file=metadata_file,
        )

    def _convert_single_to_ros1(
        self,
        bag_path: str,
        out_path: Optional[str] = None,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        validate: bool = True,
        preserve_lidar_fields: bool = False,
        lidar_topic: Optional[str] = None,
        overwrite: bool = False,
        decode_ouster: bool = False,
        decode_ffmpeg: bool = False,
        output_mode: str = "points",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        keep_raw_ouster: bool = False,
        metadata_file: Optional[str] = None,
    ) -> str:
        return _conv.convert_single_to_ros1(bag_path, out_path, include_topics, exclude_topics, remap, src_typestore, dst_typestore, validate, preserve_lidar_fields, lidar_topic, overwrite, decode_ouster, decode_ffmpeg, output_mode, points_topic, depth_topic, imu_topic, keep_raw_ouster, metadata_file)

    def _resolve_split_output_base(
        self,
        src_path: Path,
        out_path: Optional[str],
    ) -> Tuple[Path, str, Path]:
        return _conv._resolve_split_output_base(src_path, out_path)
    def _should_split_for_conversion(
        self,
        bag_path: str,
        split_duration_s: Optional[int],
        split_size_bytes: Optional[int],
    ) -> Tuple[bool, List[str]]:
        return _conv._should_split_for_conversion(bag_path, split_duration_s, split_size_bytes)
    def _discover_split_chunk_sources(self, split_root: Path) -> List[Path]:
        return _conv._discover_split_chunk_sources(split_root)
    def _storage_id_for_bag(self, bag_path: str) -> str:
        return _insp._storage_id_for_bag(bag_path)

    def _existing_split_member_sources(self, bag_path: str) -> List[Path]:
        return _insp._existing_split_member_sources(bag_path)

    def _existing_split_member_count(self, bag_path: str) -> int:
        return _insp._existing_split_member_count(bag_path)

    def _split_ros2_bag(
        self,
        bag_path: str,
        split_root: Path,
        split_duration_s: Optional[int],
        split_size_bytes: Optional[int],
    ) -> List[Path]:
        return _conv._split_ros2_bag(bag_path, split_root, split_duration_s, split_size_bytes)
    def _convert_to_ros1_with_split(
        self,
        bag_path: str,
        out_path: Optional[str] = None,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        validate: bool = True,
        preserve_lidar_fields: bool = False,
        lidar_topic: Optional[str] = None,
        overwrite: bool = False,
        split_duration_s: Optional[int] = None,
        split_size_bytes: Optional[int] = None,
        decode_ouster: bool = False,
        decode_ffmpeg: bool = False,
        output_mode: str = "points",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        keep_raw_ouster: bool = False,
        metadata_file: Optional[str] = None,
    ) -> List[str]:
        return _conv.convert_to_ros1_with_split(bag_path, out_path, include_topics, exclude_topics, remap, src_typestore, dst_typestore, validate, preserve_lidar_fields, lidar_topic, overwrite, split_duration_s, split_size_bytes, decode_ouster, decode_ffmpeg, output_mode, points_topic, depth_topic, imu_topic, keep_raw_ouster, metadata_file)
    def _convert_existing_split_members_to_ros1(
        self,
        bag_path: str,
        out_path: Optional[str] = None,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        validate: bool = True,
        preserve_lidar_fields: bool = False,
        lidar_topic: Optional[str] = None,
        overwrite: bool = False,
        decode_ouster: bool = False,
        decode_ffmpeg: bool = False,
        output_mode: str = "points",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        keep_raw_ouster: bool = False,
        metadata_file: Optional[str] = None,
    ) -> List[str]:
        return _conv.convert_existing_split_members_to_ros1(bag_path, out_path, include_topics, exclude_topics, remap, src_typestore, dst_typestore, validate, preserve_lidar_fields, lidar_topic, overwrite, decode_ouster, decode_ffmpeg, output_mode, points_topic, depth_topic, imu_topic, keep_raw_ouster, metadata_file)
    def _restore_lidar_fields_from_mcap(self, mcap_path: str, bag_path: str, topic: str) -> None:
        return _ouster._restore_lidar_fields_from_mcap(mcap_path, bag_path, topic)

    @staticmethod
    @staticmethod
    def _topic_selected(
        topic_name: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> bool:
        return _tfilter._topic_selected(topic_name, include_topics, exclude_topics)

    def _convert_single_to_ros1_with_ouster_decode(
        self,
        bag_path: str,
        out_path: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        validate: bool = True,
        overwrite: bool = False,
        decode_ouster: bool = True,
        decode_ffmpeg: bool = False,
        output_mode: str = "points",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        keep_raw_ouster: bool = False,
        metadata_file: Optional[str] = None,
    ) -> str:
        return _conv.convert_single_to_ros1_with_ouster_decode(bag_path, out_path, include_topics, exclude_topics, remap, src_typestore, dst_typestore, validate, overwrite, decode_ouster, decode_ffmpeg, output_mode, points_topic, depth_topic, imu_topic, keep_raw_ouster, metadata_file)
    def _convert_using_rosbags_api(
        self,
        src_path: str,
        dst_path: str,
        src_typestore: str,
        dst_typestore: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> None:
        return _conv._convert_using_rosbags_api(src_path, dst_path, src_typestore, dst_typestore, include_topics, exclude_topics)
    def _warn_custom_types(self, bag_path: str) -> None:
        return _valid._warn_custom_types(bag_path)

    def _validate_timestamps(self, bag_path: str, max_drift_s: float = 1.0) -> None:
        return _valid._validate_timestamps(bag_path, max_drift_s)

    def _count_ros2_messages_per_topic(self, bag_path: str) -> Dict[str, int]:
        return _valid._count_ros2_messages_per_topic(bag_path)

    def _count_ros1_messages_per_topic(self, bag_path: str) -> Dict[str, int]:
        return _valid._count_ros1_messages_per_topic(bag_path)

    @staticmethod
    def _expected_topic_counts(
        src_counts: Dict[str, int],
        include_topics: Optional[List[str]],
        exclude_topics: Optional[List[str]],
        remap: Optional[Dict[str, str]],
    ) -> Dict[str, int]:
        return _valid._expected_topic_counts(src_counts, include_topics, exclude_topics, remap)

    def _source_topic_types(self, bag_path: str) -> Dict[str, str]:
        return _tfilter._source_topic_types(bag_path)

    @staticmethod
    def _extract_parse_failed_msgtype(details: str) -> Optional[str]:
        return _tfilter._extract_parse_failed_msgtype(details)

    def _topics_for_msg_type(
        self,
        bag_path: Union[str, Path],
        msg_type: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> List[str]:
        return _tfilter._topics_for_msg_type(bag_path, msg_type, include_topics, exclude_topics)

    def _selected_topic_names(
        self,
        bag_path: Union[str, Path],
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> List[str]:
        return _tfilter._selected_topic_names(bag_path, include_topics, exclude_topics)

    @staticmethod
    def _ffmpeg_decoded_output_topic(
        input_topic: str,
        existing_topics: Optional[set] = None,
        used_topics: Optional[set] = None,
    ) -> str:
        return _tfilter._ffmpeg_decoded_output_topic(input_topic, existing_topics, used_topics)

    def _discover_ffmpeg_decode_topic_map(
        self,
        topic_types: Dict[str, str],
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> Dict[str, str]:
        return _tfilter._discover_ffmpeg_decode_topic_map(topic_types, include_topics, exclude_topics)

    def _convert_using_rosbag2_py_writer_fallback(
        self,
        src_path: Union[str, Path],
        dst_path: Union[str, Path],
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> None:
        return _conv._convert_using_rosbag2_py_writer_fallback(src_path, dst_path, src_typestore, dst_typestore, include_topics, exclude_topics)
    def _auto_exclude_unsupported_topics(
        self,
        bag_path: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> List[str]:
        return _tfilter._auto_exclude_unsupported_topics(bag_path, include_topics, exclude_topics)

    def _single_file_rosbag2_metadata(self, bag_path: str) -> Dict[str, object]:
        return _insp._single_file_rosbag2_metadata(bag_path)

    def _prepare_rosbags_convert_input(self, src_path: str, temp_root: Path) -> Path:
        return _conv._prepare_rosbags_convert_input(src_path, temp_root)
    def _rewrite_ros2_bag_for_rosbags(
        self,
        src_path: Union[str, Path],
        temp_root: Path,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> Path:
        return _conv._rewrite_ros2_bag_for_rosbags(src_path, temp_root, include_topics, exclude_topics)
    def _convert_using_rosbags_cli_or_rewrite_fallback(
        self,
        src_path: Union[str, Path],
        dst_path: Union[str, Path],
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        temp_root: Optional[Path] = None,
        rewrite_src_path: Optional[Union[str, Path]] = None,
    ) -> List[str]:
        return _conv._convert_using_rosbags_cli_or_rewrite_fallback(src_path, dst_path, src_typestore, dst_typestore, include_topics, exclude_topics, temp_root, rewrite_src_path)
    def _convert_using_rosbags_convert_fallback(
        self,
        src_path: str,
        dst_path: str,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> List[str]:
        return _conv._convert_using_rosbags_convert_fallback(src_path, dst_path, src_typestore, dst_typestore, include_topics, exclude_topics)
    def _convert_using_rosbags_cli_fallback(
        self,
        src_path: Path,
        dst_path: Path,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> None:
        return _conv._convert_using_rosbags_cli_fallback(src_path, dst_path, src_typestore, dst_typestore, include_topics, exclude_topics)
    def _first_json_string_topic(self, bag_path: str, topic_name: str) -> Optional[dict]:
        return _ouster._first_json_string_topic(bag_path, topic_name)

    @staticmethod
    def _coerce_int(value) -> Optional[int]:
        return _ouster._coerce_int(value)

    @staticmethod
    def _parse_ouster_lidar_mode(lidar_mode: Optional[str]) -> Dict[str, Optional[int]]:
        return _ouster._parse_ouster_lidar_mode(lidar_mode)

    @staticmethod
    def _parse_ouster_channel_count(model: Optional[str]) -> Optional[int]:
        return _ouster._parse_ouster_channel_count(model)

    def _ouster_packets_per_scan(self, bag_path: str, prefix: str) -> Dict[str, object]:
        return _ouster._ouster_packets_per_scan(bag_path, prefix)

    @staticmethod
    def _pick_ouster_points_topic(dst_counts: Dict[str, int], prefix: str) -> Optional[str]:
        return _ouster._pick_ouster_points_topic(dst_counts, prefix)

    def _validate_ouster_packet_topic(
        self,
        src_path: str,
        packet_topic: str,
        packet_count: int,
        dst_counts: Dict[str, int],
    ) -> bool:
        return _valid._validate_ouster_packet_topic(src_path, packet_topic, packet_count, dst_counts)

    def _validate_message_counts(
        self,
        src_path: str,
        dst_path: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
    ) -> None:
        return _valid._validate_message_counts(src_path, dst_path, include_topics, exclude_topics, remap)

    def _remap_topics(
        self,
        bag_path: Path,
        remap: Dict[str, str],
    ) -> Path:
        return _conv._remap_topics(bag_path, remap)
    def convert_folder_to_ros1(
        self,
        folder: str,
        out_folder: Optional[str] = None,
        extensions: Optional[List[str]] = None,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
        remap: Optional[Dict[str, str]] = None,
        skip_existing: bool = True,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        validate: bool = True,
        split_duration: Optional[str] = None,
        split_size: Optional[str] = None,
        decode_ouster: bool = False,
        decode_ffmpeg: bool = False,
        output_mode: str = "points",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        keep_raw_ouster: bool = False,
        metadata_file: Optional[str] = None,
    ) -> List[str]:
        """
        Convert every ROS 2 bag in *folder* to a ROS 1 .bag file.

        Parameters
        ----------
        folder          : Directory containing .mcap / .db3 files (or sub-folders
                          for split bags).
        out_folder      : Where to write the .bag files.  Defaults to *folder*.
        extensions      : File extensions to scan for.  Defaults to
                          ['.mcap', '.db3'].  Pass [''] to treat each sub-folder
                          as a bag (rosbag2 split format).
        include_topics  : Forward directly to convert_to_ros1.
        exclude_topics  : Forward directly to convert_to_ros1.
        remap           : Forward directly to convert_to_ros1.
        skip_existing   : Skip a file if the output .bag already exists.

        Returns
        -------
        List of output .bag paths that were produced.
        """
        if extensions is None:
            extensions = [".mcap", ".db3"]

        src_dir = Path(folder).resolve()
        if not src_dir.is_dir():
            raise NotADirectoryError(f"Not a directory: {src_dir}")

        dst_dir = Path(out_folder).resolve() if out_folder else src_dir
        dst_dir.mkdir(parents=True, exist_ok=True)

        # collect candidates
        candidates: List[Path] = []
        for ext in extensions:
            if ext == "":         # sub-folder bags
                candidates += [p for p in src_dir.iterdir() if p.is_dir()]
            else:
                candidates += sorted(src_dir.glob(f"*{ext}"))

        if not candidates:
            warn(f"No bags found in {src_dir} with extensions {extensions}")
            return []

        log(f"Found {len(candidates)} bag(s) in {src_dir}")
        produced: List[str] = []
        failed:   List[str] = []

        try:
            from tqdm import tqdm
            iterator = tqdm(candidates, total=len(candidates), unit="bag", dynamic_ncols=True)
        except ImportError:
            iterator = candidates

        for idx, src in enumerate(iterator, start=1):
            if hasattr(iterator, "set_description"):
                iterator.set_description(src.name[:40])
            out = dst_dir / (src.stem + "_ros1.bag")
            if skip_existing and out.exists():
                log(f"  [skip] {src.name} → {out.name} already exists")
                produced.append(str(out))
                continue
            log(f"  [{idx}/{len(candidates)}] {src.name} → {out.name}")
            try:
                result = self.convert_to_ros1(
                    str(src),
                    out_path=str(out),
                    include_topics=include_topics,
                    exclude_topics=exclude_topics,
                    remap=remap,
                    src_typestore=src_typestore,
                    dst_typestore=dst_typestore,
                    validate=validate,
                    overwrite=not skip_existing,
                    split_duration=split_duration,
                    split_size=split_size,
                    decode_ouster=decode_ouster,
                    decode_ffmpeg=decode_ffmpeg,
                    output_mode=output_mode,
                    points_topic=points_topic,
                    depth_topic=depth_topic,
                    imu_topic=imu_topic,
                    keep_raw_ouster=keep_raw_ouster,
                    metadata_file=metadata_file,
                )
                if isinstance(result, list):
                    produced.extend(result)
                else:
                    produced.append(result)
            except Exception as e:
                err(f"  FAILED {src.name}: {e}")
                failed.append(str(src))

        log(f"Done — {len(produced)} converted, {len(failed)} failed")
        if failed:
            warn("Failed files:")
            for f in failed:
                warn(f"  {f}")
        return produced

    # ---------- JSON extraction ----------
    def _get_msg_class(self, type_str: str):
        return _ouster._get_msg_class(type_str)

    def _try_extract_json(self, msg):
        for field in ("json_data", "data", "info", "payload"):
            if hasattr(msg, field):
                val = getattr(msg, field)
                if isinstance(val, str) and "{" in val:
                    try:
                        return json.loads(val)
                    except Exception:
                        continue
        return None

    def extract_json(self, bag_path: str, topic: Optional[str] = None, out_file: Optional[str] = None):
        reader = self._open_reader(bag_path)
        topics = reader.get_all_topics_and_types()
        topic_map = {t.name: t.type for t in topics}
        if topic and topic not in topic_map:
            err(f"Topic '{topic}' not found.")
            return

        results, count, json_count = [], 0, 0
        while reader.has_next():
            topic_name, data, ts = reader.read_next()
            if topic and topic_name != topic:
                continue
            cls = self._get_msg_class(topic_map[topic_name])
            if not cls:
                continue
            try:
                msg = rclpy.serialization.deserialize_message(data, cls)
                js = self._try_extract_json(msg)
                if js:
                    results.append({"timestamp": ts, "topic": topic_name, "data": js})
                    json_count += 1
            except Exception as e:
                warn(f"Failed to deserialize or parse on {topic_name}: {e}")
            count += 1

        log(f"Processed {count} msgs, extracted {json_count} JSON entries")
        if out_file:
            with open(out_file, "w") as f:
                json.dump(results, f, indent=2)
            log(f"Saved JSON output → {out_file}")
        else:
            if results:
                import pprint
                pprint.pprint(results[:3])
                if json_count > 3:
                    log(f"(showing 3 of {json_count})")
            else:
                warn("No JSON content extracted")

# --------------------------------------------------------------------------- #
#                               CLI PARSER                                    #
# --------------------------------------------------------------------------- #
def build_parser(enable_shell_completion: bool = True) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS 2 bag utilities")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    parser.add_argument("--log-file", default=None, help="Optional log file")

    sub = parser.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list_topics", help="List all topics in a bag")
    sp.add_argument("--bag", required=True)

    sp = sub.add_parser("info", help="Inspect a ROS 2 bag before conversion")
    sp.add_argument("--bag", required=True)

    sp = sub.add_parser("duration", help="Compute duration of a bag")
    sp.add_argument("--bag", required=True)

    sp = sub.add_parser("extract_json", help="Extract JSON strings from messages")
    sp.add_argument("--bag", required=True)
    sp.add_argument("--topic", help="Specific topic (optional)")
    sp.add_argument("--out", help="Output JSON file")

    sp = sub.add_parser(
        "convert_to_ros1",
        help="Convert a ROS 2 bag (.mcap/.db3) to a ROS 1 .bag",
    )
    sp.add_argument("--bag", required=True, help="Input ROS 2 bag (.mcap, .db3, or folder)")
    sp.add_argument("--out", default=None, help="Output .bag path (default: <stem>_ros1.bag)")
    sp.add_argument("--include-topic", nargs="+", default=None, metavar="TOPIC",
                    dest="include_topics", help="Only include these topics")
    sp.add_argument("--exclude-topic", nargs="+", default=None, metavar="TOPIC",
                    dest="exclude_topics", help="Exclude these topics")
    sp.add_argument("--remap", nargs="+", default=None, metavar="OLD:NEW",
                    help="Rename topics e.g. /velodyne_points:/lidar0/scan")
    sp.add_argument("--src-typestore", default="ros2_jazzy", dest="src_typestore",
                    help="rosbags source typestore (default: ros2_jazzy)")
    sp.add_argument("--dst-typestore", default="ros1_noetic", dest="dst_typestore",
                    help="rosbags destination typestore (default: ros1_noetic). "
                         "Set to 'copy' to skip field-level conversion.")
    sp.add_argument("--no-validate", action="store_true", dest="no_validate",
                    help="Skip post-conversion header.stamp vs bag-time check")
    sp.add_argument("--split-duration", default=None, dest="split_duration",
                    help="Split the source ROS2 bag before conversion when it exceeds this duration. "
                         "Accepts plain seconds or values like 90s, 5m, 1h.")
    sp.add_argument("--split-size", default=None, dest="split_size",
                    help="Split the source ROS2 bag before conversion when it exceeds this size. "
                         "Accepts bytes or values like 500M, 2G.")
    sp.add_argument("--preserve-lidar-fields", action="store_true",
                    dest="preserve_lidar_fields",
                    help="Restore custom LiDAR fields (ring, reflectivity) from MCAP")
    sp.add_argument("--lidar-topic", default=None, dest="lidar_topic",
                    help="LiDAR topic to restore fields for (default: /ouster/points/corrected)")
    sp.add_argument("--decode-ouster", action="store_true", dest="decode_ouster",
                    help="Decode Ouster packet topics and write standard ROS1 PointCloud2/Image/Imu topics directly")
    sp.add_argument("--decode-ffmpeg", action="store_true", dest="decode_ffmpeg",
                    help="Decode ffmpeg_image_transport packet topics and write standard ROS1 sensor_msgs/Image topics")
    sp.add_argument("--output-mode", choices=["points", "depth", "both"], default="points",
                    dest="output_mode",
                    help="Derived Ouster lidar output to emit when --decode-ouster is enabled")
    sp.add_argument("--points-topic", default="/ouster/points", dest="points_topic",
                    help="Output topic for decoded PointCloud2 when --decode-ouster is enabled")
    sp.add_argument("--depth-topic", default="/ouster/depth_image", dest="depth_topic",
                    help="Output topic for decoded depth Image when --decode-ouster is enabled")
    sp.add_argument("--imu-topic", default="/ouster/imu", dest="imu_topic",
                    help="Output topic for decoded Ouster Imu when --decode-ouster is enabled")
    sp.add_argument("--keep-raw-ouster", action="store_true", dest="keep_raw_ouster",
                    help="Keep raw /ouster metadata and packet topics in the ROS1 output alongside decoded topics")
    sp.add_argument("--metadata-file", default=None, dest="metadata_file",
                    help="Optional Ouster metadata file used to bootstrap split bags that do not carry /ouster/metadata")
    sp.add_argument("--overwrite", action="store_true", dest="overwrite",
                    help="Overwrite output .bag if it already exists")

    sp = sub.add_parser(
        "convert_folder",
        help="Batch-convert all ROS 2 bags (.mcap/.db3) in a folder to ROS 1 .bag",
    )
    sp.add_argument("--folder", required=True, help="Input folder containing ROS 2 bags")
    sp.add_argument("--out-folder", default=None, dest="out_folder",
                    help="Output folder (default: same as --folder)")
    sp.add_argument("--ext", nargs="+", default=None, dest="extensions",
                    metavar="EXT",
                    help="Extensions to scan, e.g. .mcap .db3 (default: .mcap .db3)")
    sp.add_argument("--include-topic", nargs="+", default=None, metavar="TOPIC",
                    dest="include_topics", help="Only include these topics")
    sp.add_argument("--exclude-topic", nargs="+", default=None, metavar="TOPIC",
                    dest="exclude_topics", help="Exclude these topics")
    sp.add_argument("--remap", nargs="+", default=None, metavar="OLD:NEW",
                    help="Rename topics e.g. /imu/data:/imu0/frame")
    sp.add_argument("--no-skip", action="store_true", dest="no_skip",
                    help="Re-convert even if output .bag already exists")
    sp.add_argument("--src-typestore", default="ros2_jazzy", dest="src_typestore",
                    help="rosbags source typestore (default: ros2_jazzy)")
    sp.add_argument("--dst-typestore", default="ros1_noetic", dest="dst_typestore",
                    help="rosbags destination typestore (default: ros1_noetic)")
    sp.add_argument("--no-validate", action="store_true", dest="no_validate",
                    help="Skip post-conversion header.stamp vs bag-time check")
    sp.add_argument("--split-duration", default=None, dest="split_duration",
                    help="Split each source bag before conversion when it exceeds this duration. "
                         "Accepts plain seconds or values like 90s, 5m, 1h.")
    sp.add_argument("--split-size", default=None, dest="split_size",
                    help="Split each source bag before conversion when it exceeds this size. "
                         "Accepts bytes or values like 500M, 2G.")
    sp.add_argument("--decode-ouster", action="store_true", dest="decode_ouster",
                    help="Decode Ouster packet topics and write standard ROS1 PointCloud2/Image/Imu topics directly")
    sp.add_argument("--decode-ffmpeg", action="store_true", dest="decode_ffmpeg",
                    help="Decode ffmpeg_image_transport packet topics and write standard ROS1 sensor_msgs/Image topics")
    sp.add_argument("--output-mode", choices=["points", "depth", "both"], default="points",
                    dest="output_mode",
                    help="Derived Ouster lidar output to emit when --decode-ouster is enabled")
    sp.add_argument("--points-topic", default="/ouster/points", dest="points_topic",
                    help="Output topic for decoded PointCloud2 when --decode-ouster is enabled")
    sp.add_argument("--depth-topic", default="/ouster/depth_image", dest="depth_topic",
                    help="Output topic for decoded depth Image when --decode-ouster is enabled")
    sp.add_argument("--imu-topic", default="/ouster/imu", dest="imu_topic",
                    help="Output topic for decoded Ouster Imu when --decode-ouster is enabled")
    sp.add_argument("--keep-raw-ouster", action="store_true", dest="keep_raw_ouster",
                    help="Keep raw /ouster metadata and packet topics in the ROS1 output alongside decoded topics")
    sp.add_argument("--metadata-file", default=None, dest="metadata_file",
                    help="Optional Ouster metadata file used to bootstrap split bags that do not carry /ouster/metadata")
    if enable_shell_completion and argcomplete:
        argcomplete.autocomplete(parser)
    return parser

# --------------------------------------------------------------------------- #
#                               MAIN ENTRY                                    #
# --------------------------------------------------------------------------- #
def main():
    args = build_parser().parse_args()

    # reconfigure shared logger according to CLI options
    global logger, log, warn, err
    logger = get_logger("Ros2utils", level=args.log_level.upper(), log_file=args.log_file)
    log, warn, err = (logger.info, logger.warning, logger.error)
    logger.debug("ros2utils argv=%s parsed_args=%s", sys.argv, vars(args))

    utils = Ros2BagUtils()

    try:
        if args.cmd == "list_topics":
            utils.list_topics(args.bag)
        elif args.cmd == "info":
            utils.info(args.bag)
        elif args.cmd == "duration":
            utils.bag_duration(args.bag)
        elif args.cmd == "extract_json":
            utils.extract_json(args.bag, args.topic, args.out)
        elif args.cmd == "convert_to_ros1":
            remap = None
            if args.remap:
                remap = {}
                for pair in args.remap:
                    if ":" not in pair:
                        err(f"--remap entry must be OLD:NEW, got: {pair}")
                        sys.exit(1)
                    old, new = pair.split(":", 1)
                    remap[old] = new
            utils.convert_to_ros1(
                args.bag,
                out_path=args.out,
                include_topics=args.include_topics,
                exclude_topics=args.exclude_topics,
                remap=remap,
                src_typestore=args.src_typestore,
                dst_typestore=args.dst_typestore,
                validate=not args.no_validate,
                preserve_lidar_fields=args.preserve_lidar_fields,
                lidar_topic=args.lidar_topic,
                split_duration=args.split_duration,
                split_size=args.split_size,
                overwrite=args.overwrite,
                decode_ouster=args.decode_ouster,
                decode_ffmpeg=args.decode_ffmpeg,
                output_mode=args.output_mode,
                points_topic=args.points_topic,
                depth_topic=args.depth_topic,
                imu_topic=args.imu_topic,
                keep_raw_ouster=args.keep_raw_ouster,
                metadata_file=args.metadata_file,
            )
        elif args.cmd == "convert_folder":
            remap = None
            if args.remap:
                remap = {}
                for pair in args.remap:
                    if ":" not in pair:
                        err(f"--remap entry must be OLD:NEW, got: {pair}")
                        sys.exit(1)
                    old, new = pair.split(":", 1)
                    remap[old] = new
            utils.convert_folder_to_ros1(
                args.folder,
                out_folder=args.out_folder,
                extensions=args.extensions,
                include_topics=args.include_topics,
                exclude_topics=args.exclude_topics,
                remap=remap,
                skip_existing=not args.no_skip,
                src_typestore=args.src_typestore,
                dst_typestore=args.dst_typestore,
                validate=not args.no_validate,
                split_duration=args.split_duration,
                split_size=args.split_size,
                decode_ouster=args.decode_ouster,
                decode_ffmpeg=args.decode_ffmpeg,
                output_mode=args.output_mode,
                points_topic=args.points_topic,
                depth_topic=args.depth_topic,
                imu_topic=args.imu_topic,
                keep_raw_ouster=args.keep_raw_ouster,
                metadata_file=args.metadata_file,
            )
        else:
            err(f"Unknown command: {args.cmd}")
    except KeyboardInterrupt:
        warn("Interrupted by user")
    except Exception as e:
        err(f"Fatal error: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
