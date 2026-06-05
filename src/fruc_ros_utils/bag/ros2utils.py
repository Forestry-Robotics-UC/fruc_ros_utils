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
        """
        Convert a ROS 2 bag (.mcap / .db3 / folder) to a ROS 1 .bag file using
        ``rosbags-convert``.

        Key differences handled
        -----------------------
        * ``--dst-typestore ros1_noetic`` triggers automatic message conversion:
            - ``builtin_interfaces/msg/Time`` → ROS1 ``time`` (sec+nsec)
            - ``sensor_msgs/CameraInfo`` field case: d/k/r/p → D/K/R/P
            - Any trivial field renames / additions / removals between distros
        * ``--src-typestore ros2_jazzy`` ensures Jazzy-specific type definitions
          are used during deserialization (not foxy defaults).
        * QoS metadata is ROS2-only and is silently stripped — correct for
          offline calibration use.
        * Custom ROS2-only types that are absent from the ROS1 typestore are
          excluded automatically instead of causing a partial or corrupt write.
        * A post-conversion sanity check compares header.stamp vs recording
          timestamps so silent time-base mismatches are caught early.

        Parameters
        ----------
        bag_path            : Path to the ROS 2 bag (.mcap, .db3, or directory).
        out_path            : Destination .bag path.  Defaults to <stem>_ros1.bag.
        include_topics      : If given, only these topics are written.
        exclude_topics      : Topics to drop (after include filter).
        remap               : {old_topic: new_topic} applied in the output bag.
        src_typestore       : rosbags source typestore (default: ros2_jazzy).
        dst_typestore       : rosbags destination typestore (default: ros1_noetic).
        validate            : Run post-conversion timestamp sanity check.
        preserve_lidar_fields : Restore custom LiDAR fields (ring, reflectivity) from MCAP.
        lidar_topic         : LiDAR topic to restore fields for (default: /ouster/points/corrected).
        overwrite           : Remove existing output path before writing if true.
        decode_ouster       : Decode Ouster packet topics to standard sensor topics during conversion.
        decode_ffmpeg       : Decode ffmpeg_image_transport packet topics to sensor_msgs/Image during conversion.
        """
        src = Path(bag_path).resolve()
        if not src.exists():
            raise FileNotFoundError(f"Bag not found: {src}")

        if out_path is None:
            stem = src.stem if src.is_file() else src.name
            out_path = src.parent / f"{stem}_ros1.bag"
        dst = Path(out_path).resolve()
        self._ensure_output_available(str(dst), overwrite=overwrite)
        logger.debug(
            "convert_to_ros1 request: src=%s dst=%s include=%s exclude=%s remap=%s validate=%s preserve_lidar_fields=%s lidar_topic=%s overwrite=%s decode_ouster=%s decode_ffmpeg=%s output_mode=%s keep_raw_ouster=%s metadata_file=%s",
            _debug_path_state(src),
            _debug_path_state(dst),
            include_topics,
            exclude_topics,
            remap,
            validate,
            preserve_lidar_fields,
            lidar_topic,
            overwrite,
            decode_ouster,
            decode_ffmpeg,
            output_mode,
            keep_raw_ouster,
            metadata_file,
        )
        self._debug_log_ros2_bag_summary(src, "convert_to_ros1 source summary")

        if decode_ouster or decode_ffmpeg:
            if decode_ouster and preserve_lidar_fields:
                warn("Ignoring --preserve-lidar-fields because --decode-ouster writes derived ROS1 topics directly.")
            converted_path = self._convert_single_to_ros1_with_ouster_decode(
                bag_path=str(src),
                out_path=str(dst),
                include_topics=include_topics,
                exclude_topics=exclude_topics,
                remap=remap,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                validate=validate,
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
            return converted_path

        # Warn about custom types that won't be in the standard typestore
        self._warn_custom_types(str(src))
        auto_excluded_topics = self._auto_exclude_unsupported_topics(
            str(src),
            include_topics=include_topics,
            exclude_topics=exclude_topics,
        )
        effective_exclude_topics = _merge_topic_lists(exclude_topics, auto_excluded_topics)
        if auto_excluded_topics:
            warn(
                "Automatically excluding unsupported ROS2 topics from ROS1 conversion: %s",
                ", ".join(auto_excluded_topics),
            )

        log(f"Converting {src} → {dst}")
        log(f"src-typestore={src_typestore}  dst-typestore={dst_typestore}")

        # Use the rosbags conversion path only (API + CLI fallbacks).
        runtime_excluded_topics = self._convert_using_rosbags_convert_fallback(
            str(src),
            str(dst),
            src_typestore=src_typestore,
            dst_typestore=dst_typestore,
            include_topics=include_topics,
            exclude_topics=effective_exclude_topics,
        )
        effective_exclude_topics = _merge_topic_lists(
            effective_exclude_topics,
            runtime_excluded_topics,
        )
        if runtime_excluded_topics:
            warn(
                "Additionally excluding topics after rewritten sqlite3 parse failures: %s",
                ", ".join(runtime_excluded_topics),
            )
        log(f"Conversion complete → {dst}")

        if preserve_lidar_fields and dst.exists():
            if lidar_topic is None:
                lidar_topic = "/ouster/points/corrected"
            log(f"Restoring custom LiDAR fields on '{lidar_topic}'...")
            try:
                self._restore_lidar_fields_from_mcap(str(src), str(dst), lidar_topic)
                log("LiDAR fields restored successfully")
            except Exception as e:
                warn(f"Failed to restore LiDAR fields: {e}")
        if remap:
            dst = self._remap_topics(dst, remap)

        if validate and dst.exists():
            self._validate_timestamps(str(dst))
            self._validate_message_counts(
                src_path=str(src),
                dst_path=str(dst),
                include_topics=include_topics,
                exclude_topics=effective_exclude_topics,
                remap=remap,
            )

        return str(dst)

    def _resolve_split_output_base(
        self,
        src_path: Path,
        out_path: Optional[str],
    ) -> Tuple[Path, str, Path]:
        if out_path is None:
            out_dir = src_path.parent
            base_stem = f"{src_path.stem}_ros1"
            single_out = out_dir / f"{base_stem}.bag"
        else:
            out_candidate = Path(out_path).resolve()
            if out_candidate.suffix:
                out_dir = out_candidate.parent
                base_stem = out_candidate.stem
                single_out = out_candidate
            else:
                out_dir = out_candidate
                base_stem = f"{src_path.stem}_ros1"
                single_out = out_dir / f"{base_stem}.bag"
        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir, base_stem, single_out

    def _should_split_for_conversion(
        self,
        bag_path: str,
        split_duration_s: Optional[int],
        split_size_bytes: Optional[int],
    ) -> Tuple[bool, List[str]]:
        reasons: List[str] = []
        if split_duration_s:
            duration_s = self.bag_duration(bag_path)
            if duration_s > float(split_duration_s):
                reasons.append(
                    f"duration {duration_s:.3f}s exceeds limit {int(split_duration_s)}s"
                )
        if split_size_bytes:
            size_bytes = self.bag_size_bytes(bag_path)
            if size_bytes > int(split_size_bytes):
                reasons.append(
                    f"size {_format_bytes(size_bytes)} exceeds limit {_format_bytes(int(split_size_bytes))}"
                )
        return bool(reasons), reasons

    def _discover_split_chunk_sources(self, split_root: Path) -> List[Path]:
        chunks = [
            path
            for path in split_root.rglob("*")
            if path.is_file() and path.suffix.lower() in (".mcap", ".db3")
        ]
        return sorted(chunks, key=_natural_sort_key)

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
        if not split_duration_s and not split_size_bytes:
            return [Path(bag_path).resolve()]
        if rosbag2_py is None:
            raise RuntimeError(
                "rosbag2_py is required to split ROS 2 bags in this environment. "
                "Use the jazzy container/image."
            )

        split_root.mkdir(parents=True, exist_ok=True)
        output_prefix = split_root / Path(bag_path).stem
        storage_id = self._storage_id_for_bag(bag_path)

        storage_options = rosbag2_py.StorageOptions(
            uri=str(output_prefix),
            storage_id=storage_id,
        )
        if split_duration_s:
            storage_options.max_bagfile_duration = int(split_duration_s)
        if split_size_bytes:
            storage_options.max_bagfile_size = int(split_size_bytes)

        log(
            "Splitting source bag before conversion via rosbag2_py: %s -> %s",
            bag_path,
            output_prefix,
        )
        reader = self._open_reader(bag_path)
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, rosbag2_py.ConverterOptions("", ""))

        try:
            for topic_meta in reader.get_all_topics_and_types():
                writer.create_topic(topic_meta)

            messages_written = 0
            while reader.has_next():
                topic_name, data, ts = reader.read_next()
                writer.write(topic_name, data, ts)
                messages_written += 1
        finally:
            close_writer = getattr(writer, "close", None)
            if callable(close_writer):
                close_writer()
            close_reader = getattr(reader, "close", None)
            if callable(close_reader):
                close_reader()
            del writer
            del reader

        chunks = self._discover_split_chunk_sources(split_root)
        if not chunks:
            raise RuntimeError(
                f"ROS 2 bag split produced no .mcap/.db3 chunks under {split_root}"
            )
        log(
            "Split produced %d chunk source file(s) from %d message(s)",
            len(chunks),
            messages_written,
        )
        return chunks

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
        src = Path(bag_path).resolve()
        if not src.exists():
            raise FileNotFoundError(f"Bag not found: {src}")

        out_dir, base_stem, single_out = self._resolve_split_output_base(src, out_path)
        should_split, reasons = self._should_split_for_conversion(
            str(src),
            split_duration_s=split_duration_s,
            split_size_bytes=split_size_bytes,
        )
        if not should_split:
            log("Split thresholds not triggered; converting source bag directly.")
            return [
                self._convert_single_to_ros1(
                    bag_path=str(src),
                    out_path=str(single_out),
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
            ]

        log("Chunked conversion requested because %s", "; ".join(reasons))
        produced: List[str] = []
        with tempfile.TemporaryDirectory(
            prefix=f".ros2_split_{src.stem}_",
            dir=str(out_dir),
            ignore_cleanup_errors=True,
        ) as temp_dir:
            chunk_sources = self._split_ros2_bag(
                bag_path=str(src),
                split_root=Path(temp_dir),
                split_duration_s=split_duration_s,
                split_size_bytes=split_size_bytes,
            )

            pbar = _make_progress_bar(
                "Converting split chunks",
                "chunk",
                total=len(chunk_sources),
                position=0,
                leave=True,
            )
            try:
                for idx, chunk_source in enumerate(chunk_sources):
                    if pbar is not None:
                        pbar.update(1)
                    chunk_out = out_dir / f"{base_stem}_chunk_{idx:03d}.bag"
                    log(
                        "Chunk %d/%d: %s -> %s",
                        idx + 1,
                        len(chunk_sources),
                        chunk_source.name,
                        chunk_out.name,
                    )
                    produced.append(
                        self._convert_single_to_ros1(
                            bag_path=str(chunk_source),
                            out_path=str(chunk_out),
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
                    )
            finally:
                if pbar is not None:
                    pbar.close()

        log("Chunked conversion complete: %d ROS1 bag(s) written to %s", len(produced), out_dir)
        return produced

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
        src = Path(bag_path).resolve()
        out_dir, base_stem, single_out = self._resolve_split_output_base(src, out_path)
        with tempfile.TemporaryDirectory(
            prefix=f".ros2_existing_split_{src.stem}_",
            dir=str(out_dir),
            ignore_cleanup_errors=True,
        ):
            chunk_sources = self._existing_split_member_sources(str(src))
            if len(chunk_sources) <= 1:
                return [
                    self._convert_single_to_ros1(
                        bag_path=str(chunk_sources[0]),
                        out_path=str(single_out),
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
                ]

            log(
                "Source ROS2 bag already contains %d split member files; preserving that split layout in ROS1 output.",
                len(chunk_sources),
            )
            produced: List[str] = []
            pbar = _make_progress_bar(
                "Converting existing split members",
                "chunk",
                total=len(chunk_sources),
                position=0,
                leave=True,
            )
            try:
                for idx, chunk_source in enumerate(chunk_sources):
                    if pbar is not None:
                        pbar.update(1)
                    chunk_out = out_dir / f"{base_stem}_chunk_{idx:03d}.bag"
                    produced.append(
                        self._convert_single_to_ros1(
                            bag_path=str(chunk_source),
                            out_path=str(chunk_out),
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
                    )
            finally:
                if pbar is not None:
                    pbar.close()
        return produced

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
        try:
            from rosbags.rosbag1 import Writer as Ros1Writer
            from rosbags.serde import cdr_to_ros1
        except ImportError as exc:
            raise RuntimeError(
                f"rosbags rosbag1 writer/serde support is required for direct decode conversion: {exc}"
            ) from exc

        src = Path(bag_path).resolve()
        dst = Path(out_path).resolve()
        self._ensure_output_available(str(dst), overwrite=overwrite)

        src_store = self._resolve_rosbags_typestore(src_typestore)
        dst_store = self._resolve_rosbags_typestore(dst_typestore)
        self._ensure_ros1_tf2_typestore(dst_store)

        source_topic_types = self._source_topic_types(str(src))
        raw_ouster_topics: set = set()
        raw_ffmpeg_topics: set = set()
        derived_topic_types: Dict[str, str] = {}
        ffmpeg_topic_map: Dict[str, str] = {}

        ouster_decoder = None
        if decode_ouster:
            from fruc_ros_utils.bag.ouster_decode import OusterPacketDecoder

            ouster_decoder = OusterPacketDecoder(
                points_topic=points_topic,
                depth_topic=depth_topic,
                imu_topic=imu_topic,
                output_mode=output_mode,
                metadata_file=metadata_file,
                input_bag_path=str(src),
            )
            ouster_decoder.initialize_metadata_fallback_if_available()
            raw_ouster_topics = set(ouster_decoder.raw_topics())
            derived_topic_types.update(ouster_decoder.output_topic_types())

        ffmpeg_decoder = None
        if decode_ffmpeg:
            ffmpeg_topic_map = self._discover_ffmpeg_decode_topic_map(
                source_topic_types,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
            )
            if ffmpeg_topic_map:
                from fruc_ros_utils.bag.ffmpeg_decode import FFMPEGPacketDecoder

                ffmpeg_decoder = FFMPEGPacketDecoder(topic_map=ffmpeg_topic_map)
                raw_ffmpeg_topics = set(ffmpeg_decoder.raw_topics())
                derived_topic_types.update(ffmpeg_decoder.output_topic_types())
                log(
                    "FFmpeg decode enabled for %d topic(s): %s",
                    len(ffmpeg_topic_map),
                    ", ".join(
                        f"{src_topic}->{dst_topic}"
                        for src_topic, dst_topic in sorted(ffmpeg_topic_map.items())
                    ),
                )
            else:
                warn("FFmpeg decode requested but no ffmpeg packet topics were selected.")

        auto_excluded_topics = self._auto_exclude_unsupported_topics(
            str(src),
            include_topics=include_topics,
            exclude_topics=exclude_topics,
        )
        if auto_excluded_topics:
            warn(
                "Automatically excluding unsupported ROS2 topics from direct ROS1 conversion: %s",
                ", ".join(auto_excluded_topics),
            )

        base_exclude_topics = list(_merge_topic_lists(exclude_topics, auto_excluded_topics) or [])
        if raw_ffmpeg_topics:
            base_exclude_topics = list(_merge_topic_lists(base_exclude_topics, list(raw_ffmpeg_topics)) or [])
        if not keep_raw_ouster:
            base_exclude_topics = list(_merge_topic_lists(base_exclude_topics, list(raw_ouster_topics)) or [])

        selected_copy_topics = self._selected_topic_names(
            str(src),
            include_topics=include_topics,
            exclude_topics=base_exclude_topics,
        )

        include_set = set(include_topics or [])
        exclude_set = set(exclude_topics or [])
        ffmpeg_output_to_input = {dst_topic: src_topic for src_topic, dst_topic in ffmpeg_topic_map.items()}
        selected_derived_topics: Dict[str, str] = {}
        for topic_name, msg_type in derived_topic_types.items():
            selected = self._topic_selected(
                topic_name,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
            )
            if (
                not selected
                and include_set
                and topic_name in ffmpeg_output_to_input
                and ffmpeg_output_to_input[topic_name] in include_set
                and ffmpeg_output_to_input[topic_name] not in exclude_set
            ):
                selected = True
            if selected:
                selected_derived_topics[topic_name] = msg_type

        if not selected_copy_topics and not selected_derived_topics:
            raise RuntimeError(
                "No ROS1 output topics selected after applying include/exclude filters and decode settings."
            )

        self._cleanup_partial_output(dst, context="direct decode ROS1 writer preflight")
        logger.debug(
            "Starting direct ROS2->ROS1 conversion with decoders: src=%s dst=%s decode_ouster=%s decode_ffmpeg=%s copy_topics=%s derived_topics=%s",
            _debug_path_state(src),
            _debug_path_state(dst),
            decode_ouster,
            decode_ffmpeg,
            selected_copy_topics,
            selected_derived_topics,
        )

        decoders = []
        if ouster_decoder is not None:
            decoders.append(("Ouster", ouster_decoder))
        if ffmpeg_decoder is not None:
            decoders.append(("FFmpeg", ffmpeg_decoder))

        reader = self._open_reader(str(src))
        close_reader = getattr(reader, "close", None)
        copy_connections = {}
        derived_connections = {}
        msg_count = 0
        reader_metadata = getattr(reader, "get_metadata", lambda: None)()
        reader_total = int(getattr(reader_metadata, "message_count", 0) or 0) or None
        pbar = _make_progress_bar(
            "Direct ROS2->ROS1 decode",
            "msg",
            total=reader_total,
            position=1,
            leave=False,
        )
        try:
            with Ros1Writer(dst) as writer:
                for topic_name in selected_copy_topics:
                    msg_type = source_topic_types[topic_name]
                    out_topic_name = remap.get(topic_name, topic_name) if remap else topic_name
                    copy_connections[topic_name] = writer.add_connection(
                        out_topic_name,
                        msg_type,
                        typestore=dst_store,
                        latching=1 if topic_name.endswith("/tf_static") else None,
                    )

                for topic_name, msg_type in selected_derived_topics.items():
                    out_topic_name = remap.get(topic_name, topic_name) if remap else topic_name
                    derived_connections[topic_name] = writer.add_connection(
                        out_topic_name,
                        msg_type,
                        typestore=dst_store,
                        latching=1 if topic_name.endswith("/tf_static") else None,
                    )

                while reader.has_next():
                    if pbar is not None:
                        pbar.update(1)
                    topic_name, rawdata, timestamp = reader.read_next()
                    timestamp = int(timestamp)

                    connection = copy_connections.get(topic_name)
                    if connection is not None:
                        msg_type = source_topic_types[topic_name]
                        ros1_raw = cdr_to_ros1(rawdata, msg_type, typestore=src_store)
                        writer.write(connection, timestamp, ros1_raw)
                        msg_count += 1

                    for decoder_name, decoder in decoders:
                        try:
                            derived_outputs = decoder.process_message(topic_name, rawdata, timestamp)
                        except Exception as exc:
                            warn(
                                "Non-fatal %s decode error in %s at %d on topic %s: %s",
                                decoder_name,
                                src.name,
                                timestamp,
                                topic_name,
                                exc,
                            )
                            derived_outputs = []

                        for derived_topic, derived_type, derived_msg, derived_ts in derived_outputs:
                            derived_connection = derived_connections.get(derived_topic)
                            if derived_connection is None:
                                continue
                            derived_cdr = rclpy.serialization.serialize_message(derived_msg)
                            derived_ros1 = cdr_to_ros1(derived_cdr, derived_type, typestore=src_store)
                            writer.write(derived_connection, int(derived_ts), derived_ros1)
                            msg_count += 1
        finally:
            if pbar is not None:
                pbar.close()
            if callable(close_reader):
                close_reader()

        if ouster_decoder is not None:
            decode_stats = ouster_decoder.get_stats()
            log(
                "Ouster decode summary for %s: metadata=%d fallback=%d lidar_packets=%d scans=%d depth=%d imu_packets=%d imu_msgs=%d skipped=%d decode_errors=%d",
                src.name,
                decode_stats.get("metadata_messages", 0),
                decode_stats.get("metadata_fallback_used", 0),
                decode_stats.get("lidar_packets_seen", 0),
                decode_stats.get("scans_emitted", 0),
                decode_stats.get("depth_images_emitted", 0),
                decode_stats.get("imu_packets_seen", 0),
                decode_stats.get("imu_messages_emitted", 0),
                decode_stats.get("messages_skipped", 0),
                decode_stats.get("decode_errors", 0),
            )
        if ffmpeg_decoder is not None:
            ffmpeg_stats = ffmpeg_decoder.get_stats()
            log(
                "FFmpeg decode summary for %s: packets=%d frames=%d no_frame=%d decode_errors=%d",
                src.name,
                ffmpeg_stats.get("packets_seen", 0),
                ffmpeg_stats.get("frames_emitted", 0),
                ffmpeg_stats.get("packets_without_frames", 0),
                ffmpeg_stats.get("decode_errors", 0),
            )

        if msg_count == 0:
            self._cleanup_partial_output(dst, context="direct decode wrote zero messages")
            raise RuntimeError("Direct ROS2->ROS1 decode conversion wrote zero messages.")

        final_dst = dst

        if validate and final_dst.exists():
            validation_excludes = list(base_exclude_topics)
            validation_excludes = list(_merge_topic_lists(validation_excludes, list(raw_ouster_topics)) or [])
            validation_excludes = list(_merge_topic_lists(validation_excludes, list(raw_ffmpeg_topics)) or [])
            if decode_ouster and output_mode in ("depth", "both"):
                validation_excludes = list(_merge_topic_lists(validation_excludes, [depth_topic]) or [])
            self._validate_timestamps(str(final_dst))
            self._validate_message_counts(
                src_path=str(src),
                dst_path=str(final_dst),
                include_topics=include_topics,
                exclude_topics=validation_excludes,
                remap=remap,
            )

        return str(final_dst)

    def _convert_using_rosbags_api(
        self,
        src_path: str,
        dst_path: str,
        src_typestore: str,
        dst_typestore: str,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> None:
        """
        Convert a ROS 2 bag JAZZY to ROS 1 using rosbags Python API directly.
        
        This avoids the CLI tool and is more robust against serialization edge cases.

        KEY IMPROVEMENT: Catches KeyError during Ros2Reader initialization (which means
        the MCAP has malformed schema encoding) and immediately falls back to the
        rosbags conversion fallback path, without attempting raw-byte deserialization.

        This prevents the error from propagating and allows the fallback path to execute.
        """
        try:
            from rosbags.rosbag2 import Reader as Ros2Reader
            from rosbags.rosbag1 import Writer as Ros1Writer
        except ImportError as e:
            raise RuntimeError(
                f"rosbags library components not available: {e}. "
                "Install the repo default version, e.g. "
                "pip install 'rosbags==0.10.11'"
            )

        # CRITICAL: Try to initialize Ros2Reader. If it fails with KeyError on schema
        # encoding, we know the MCAP is corrupted and should use the rosbags fallback.
        # This exception happens at __enter__/__init__ time, before we try reading messages.
        reader2 = None
        reader2_ok = False
        try:
            reader2 = Ros2Reader(src_path)
            reader2_ok = True
        except KeyError as ke:
            # The MCAP file has at least one message type with an empty or malformed
            # encoding field in its schema definition. The rosbags library cannot parse
            # this, but we can still attempt recovery through the rosbags fallback path.
            logger.debug(
                "Ros2Reader.__init__ failed with KeyError (malformed MCAP schema encoding): %s. "
                "Switching to rosbags convert fallback.", ke
            )
            warn(
                f"MCAP schema corruption detected (KeyError on empty encoding). "
                f"Using rosbags conversion fallback."
            )
            return self._convert_using_rosbags_convert_fallback(
                src_path,
                dst_path,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
            )
        except Exception as e:
            # Some other initialization error — re-raise for the caller to handle
            logger.debug("Ros2Reader initialization failed with non-KeyError: %s", e, exc_info=True)
            raise RuntimeError(f"Ros2Reader initialization failed: {e}")

        # If we reach here, Ros2Reader opened successfully. Use it for full conversion.
        if not reader2_ok:
            raise RuntimeError("Ros2Reader should be open but reader2_ok is False")

        try:
            with reader2 as reader:
                # Ensure destination parent directory exists so Ros1Writer can open the file
                dst_parent = Path(dst_path).parent
                if not dst_parent.exists():
                    logger.debug("Creating destination directory: %s", dst_parent)
                    dst_parent.mkdir(parents=True, exist_ok=True)
                
                with Ros1Writer(dst_path) as writer1:
                    # Get all topics from reader connections
                    all_topics = {conn.topic: conn.msgtype for conn in reader.connections}
                    logger.debug("rosbags reader connections: %d topics found", len(all_topics))
                    for t, mt in all_topics.items():
                        logger.debug(" - %s : %s", t, mt)

                    # Filter topics if requested
                    topics_to_write = set(all_topics.keys())
                    if include_topics:
                        topics_to_write &= set(include_topics)
                        logger.debug("After include filter: %d topics", len(topics_to_write))
                    if exclude_topics:
                        topics_to_write -= set(exclude_topics)
                        logger.debug("After exclude filter: %d topics", len(topics_to_write))

                    # Convert and write messages
                    msg_count = 0
                    pbar = _make_progress_bar("Converting ROS2->ROS1", "msg")
                    try:
                        for conn, _, rawdata in reader.messages():
                            if pbar is not None:
                                pbar.update(1)
                            if conn.topic not in topics_to_write:
                                continue

                            # Attempt to deserialize with fallback to raw bytes if deserialization fails
                            try:
                                msg = reader.deserialize(rawdata, conn.msgtype)
                                try:
                                    writer1.write(conn.topic, msg, int(conn.timestamp))
                                    msg_count += 1
                                except Exception as e:
                                    warn(f"Failed to write deserialized msg on {conn.topic}: {e}; "
                                         f"writing raw bytes instead")
                                    logger.debug("write-deserialized exception for topic %s: %s",
                                                 conn.topic, e, exc_info=True)
                                    try:
                                        writer1.write(conn.topic, conn.msgtype, rawdata, int(conn.timestamp))
                                        msg_count += 1
                                    except Exception as e2:
                                        logger.debug("write-raw exception for topic %s: %s",
                                                     conn.topic, e2, exc_info=True)
                                        raise RuntimeError(
                                            f"Failed to write topic {conn.topic} at {int(conn.timestamp)} "
                                            f"using both converted and raw message paths. "
                                            f"deserialized_write_error={e}; raw_write_error={e2}"
                                        ) from e2
                            except Exception as e:
                                warn(f"Failed to deserialize on {conn.topic}: {e}; "
                                     f"writing raw bytes instead")
                                logger.debug("deserialize exception for topic %s: %s",
                                             conn.topic, e, exc_info=True)
                                try:
                                    writer1.write(conn.topic, conn.msgtype, rawdata, int(conn.timestamp))
                                    msg_count += 1
                                except Exception as e2:
                                    logger.debug("write-raw exception for topic %s: %s",
                                                 conn.topic, e2, exc_info=True)
                                    raise RuntimeError(
                                        f"Failed to convert topic {conn.topic} at {int(conn.timestamp)} "
                                        f"after deserialization failure. "
                                        f"deserialize_error={e}; raw_write_error={e2}"
                                    ) from e2
                                continue
                    finally:
                        if pbar is not None:
                            pbar.close()

                    log(f"Converted {msg_count} messages using rosbags deserialization")
        except KeyError as ke:
            # Some malformed MCAP schemas only fail when entering the reader context
            # (during schema parsing), not during Reader construction.
            warn(
                "MCAP schema corruption detected while opening Ros2Reader context "
                f"(KeyError: {ke}). Using rosbags convert fallback."
            )
            return self._convert_using_rosbags_convert_fallback(
                src_path,
                dst_path,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
            )
        except Exception as e:
            logger.debug("Exception during rosbags conversion: %s", e, exc_info=True)
            raise


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
        """
        Convert directly from a ROS2 bag opened with ``rosbag2_py`` to a ROS1 bag.

        This bypasses rosbags' rosbag2 readers entirely and is used when the
        converter can no longer reopen rewritten sqlite3 bags due typestore
        digest assertions on otherwise standard ROS message types.
        """
        try:
            from rosbags.rosbag1 import Writer as Ros1Writer
            from rosbags.serde import cdr_to_ros1
        except ImportError as exc:
            raise RuntimeError(
                f"rosbags rosbag1 writer/serde support is required for direct fallback: {exc}"
            ) from exc

        src_store = self._resolve_rosbags_typestore(src_typestore)
        dst_store = self._resolve_rosbags_typestore(dst_typestore)
        self._ensure_ros1_tf2_typestore(dst_store)
        selected_topics = self._selected_topic_names(
            src_path,
            include_topics=include_topics,
            exclude_topics=exclude_topics,
        )
        if not selected_topics:
            raise RuntimeError(
                "Direct rosbag2_py -> rosbag1 fallback has no selected topics to convert."
            )

        self._cleanup_partial_output(
            dst_path,
            context="rosbag2_py direct ROS1 writer preflight",
        )
        logger.debug(
            "Starting direct rosbag2_py -> rosbag1 writer fallback: src=%s dst=%s include=%s exclude=%s",
            _debug_path_state(src_path),
            _debug_path_state(dst_path),
            include_topics,
            exclude_topics,
        )
        self._debug_log_ros2_bag_summary(src_path, "Direct rosbag2_py fallback source summary")

        reader = self._open_reader(str(src_path))
        close_reader = getattr(reader, "close", None)
        connections = {}
        topic_types = self._source_topic_types(str(src_path))
        msg_count = 0
        reader_metadata = getattr(reader, "get_metadata", lambda: None)()
        reader_total = int(getattr(reader_metadata, "message_count", 0) or 0) or None
        pbar = _make_progress_bar(
            "Direct ROS2->ROS1 fallback",
            "msg",
            total=reader_total,
            position=1,
            leave=False,
        )
        try:
            with Ros1Writer(dst_path) as writer:
                for topic_name in selected_topics:
                    msg_type = topic_types[topic_name]
                    try:
                        connections[topic_name] = writer.add_connection(
                            topic_name,
                            msg_type,
                            typestore=dst_store,
                            latching=1 if topic_name.endswith("/tf_static") else None,
                        )
                    except Exception as exc:
                        raise RuntimeError(
                            f"Failed to register ROS1 connection for topic {topic_name} ({msg_type}): {exc}"
                        ) from exc

                while reader.has_next():
                    if pbar is not None:
                        pbar.update(1)
                    topic_name, rawdata, timestamp = reader.read_next()
                    connection = connections.get(topic_name)
                    if connection is None:
                        continue
                    msg_type = topic_types[topic_name]
                    try:
                        ros1_raw = cdr_to_ros1(rawdata, msg_type, typestore=src_store)
                    except Exception as exc:
                        raise RuntimeError(
                            f"Failed to convert CDR to ROS1 for topic {topic_name} ({msg_type}) at {int(timestamp)}: {exc}"
                        ) from exc
                    try:
                        writer.write(connection, int(timestamp), ros1_raw)
                    except Exception as exc:
                        raise RuntimeError(
                            f"Failed to write ROS1 message for topic {topic_name} ({msg_type}) at {int(timestamp)}: {exc}"
                        ) from exc
                    msg_count += 1
        finally:
            if pbar is not None:
                pbar.close()
            if callable(close_reader):
                close_reader()

        if msg_count == 0:
            self._cleanup_partial_output(
                dst_path,
                context="rosbag2_py direct ROS1 writer wrote zero messages",
            )
            raise RuntimeError(
                "Direct rosbag2_py -> rosbag1 fallback wrote zero messages."
            )
        logger.debug(
            "Direct rosbag2_py -> rosbag1 writer fallback wrote %d message(s) to %s",
            msg_count,
            _debug_path_state(dst_path),
        )

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
        src = Path(src_path).resolve()
        logger.debug(
            "Preparing rosbags convert input from %s into temp_root=%s",
            _debug_path_state(src),
            _debug_path_state(temp_root),
        )
        if src.is_dir():
            logger.debug("rosbags convert input already directory-backed; using source directly")
            return src

        temp_root.mkdir(parents=True, exist_ok=True)
        wrapped = temp_root / f"{src.stem}_rosbag2"
        wrapped.mkdir(parents=True, exist_ok=True)
        staged_src = wrapped / src.name
        if not staged_src.exists():
            try:
                staged_src.symlink_to(src)
            except (OSError, PermissionError):
                try:
                    os.link(src, staged_src)
                except (OSError, PermissionError):
                    logger.debug(
                        "Hard link also failed, copying file instead: %s",
                        src,
                    )
                    shutil.copy2(src, staged_src)
        metadata_path = wrapped / "metadata.yaml"
        metadata_path.write_text(
            yaml.safe_dump(
                self._single_file_rosbag2_metadata(str(src)),
                sort_keys=False,
                default_flow_style=False,
            ),
            encoding="utf-8",
        )
        logger.debug(
            "Wrapped standalone ROS2 bag for rosbags.convert: wrapped=%s staged_src=%s metadata=%s",
            _debug_path_state(wrapped),
            _debug_path_state(staged_src),
            _debug_path_state(metadata_path),
        )
        return wrapped

    def _rewrite_ros2_bag_for_rosbags(
        self,
        src_path: Union[str, Path],
        temp_root: Path,
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> Path:
        """
        Normalize a ROS2 bag through ``ros2 bag convert`` into sqlite3 storage.

        This is used only when rosbags fails to parse an MCAP schema entry
        (e.g. ``KeyError('')`` from an empty schema encoding). Rewriting via the
        ROS 2 CLI avoids rosbags' MCAP parser while keeping the subsequent
        ROS2->ROS1 conversion on the rosbags path.
        """
        src = Path(src_path)
        temp_root.mkdir(parents=True, exist_ok=True)
        rewritten_dir = temp_root / f"{src.stem}_sqlite3"
        self._cleanup_partial_output(rewritten_dir, context="ros2 bag convert rewrite preflight")
        logger.debug(
            "Rewriting ROS2 bag for rosbags compatibility: src=%s rewritten_dir=%s include=%s exclude=%s",
            _debug_path_state(src),
            _debug_path_state(rewritten_dir),
            include_topics,
            exclude_topics,
        )
        self._debug_log_ros2_bag_summary(src, "Pre-rewrite ROS2 source summary")

        input_storage_id = ""
        if src.is_file():
            input_storage_id = {
                ".mcap": "mcap",
                ".db3": "sqlite3",
            }.get(src.suffix.lower(), "")

        output_options = {
            "output_bags": [
                {
                    "uri": str(rewritten_dir),
                    "storage_id": "sqlite3",
                    "all_topics": not bool(include_topics),
                    "topics": include_topics or [],
                    "exclude_topics": exclude_topics or [],
                }
            ]
        }
        output_options_path = temp_root / "ros2_bag_convert_output.yaml"
        output_options_path.write_text(
            yaml.safe_dump(
                output_options,
                sort_keys=False,
                default_flow_style=False,
            ),
            encoding="utf-8",
        )
        logger.debug(
            "ros2 bag convert rewrite config written to %s: %s",
            output_options_path,
            output_options,
        )

        cmd = [
            "ros2",
            "bag",
            "convert",
            "-i",
            str(src),
        ]
        if input_storage_id:
            cmd.append(input_storage_id)
        cmd.extend(["-o", str(output_options_path)])
        logger.debug("Running ROS2 bag rewrite for rosbags compatibility: %s", cmd)
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
        )
        self._debug_log_subprocess_result("ros2 bag convert rewrite", cmd, result)
        if result.returncode != 0:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            details = stderr or stdout or f"exit code {result.returncode}"
            raise RuntimeError(f"ros2 bag convert rewrite failed: {details}")
        if not rewritten_dir.exists():
            raise RuntimeError(
                f"ros2 bag convert rewrite reported success but did not create {rewritten_dir}"
            )
        logger.debug("Rewrite created %s", _debug_path_state(rewritten_dir))
        self._debug_log_ros2_bag_summary(rewritten_dir, "Post-rewrite ROS2 source summary")
        return rewritten_dir

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
        """Run ``rosbags-convert`` and recover from reader-specific schema/type parse failures."""
        src = Path(src_path)
        dst = Path(dst_path)
        try:
            self._convert_using_rosbags_cli_fallback(
                src_path=src,
                dst_path=dst,
                src_typestore=src_typestore,
                dst_typestore=dst_typestore,
                include_topics=include_topics,
                exclude_topics=exclude_topics,
            )
            return []
        except RuntimeError as exc:
            details = str(exc)
            if "schema KeyError" not in details:
                raise

        warn(
            "rosbags-convert CLI reported schema KeyError; "
            "rewriting source bag with ros2 bag convert to sqlite3 and retrying."
        )
        rewrite_root = temp_root or (dst.parent / ".ros2_bag_convert_rewrite")
        rewrite_src = Path(rewrite_src_path) if rewrite_src_path is not None else src
        logger.debug(
            "CLI fallback rewrite parameters: rewrite_root=%s rewrite_src=%s dst=%s",
            _debug_path_state(rewrite_root),
            _debug_path_state(rewrite_src),
            _debug_path_state(dst),
        )
        active_src = rewrite_src
        active_exclude_topics = list(exclude_topics or [])
        dynamically_excluded_topics: List[str] = []

        for attempt_idx in range(8):
            rewritten_src = self._rewrite_ros2_bag_for_rosbags(
                src_path=active_src,
                temp_root=Path(rewrite_root),
                include_topics=include_topics,
                exclude_topics=active_exclude_topics,
            )
            self._cleanup_partial_output(
                dst,
                context=f"retry after ros2 bag convert rewrite attempt {attempt_idx + 1}",
            )
            try:
                self._convert_using_rosbags_cli_fallback(
                    src_path=rewritten_src,
                    dst_path=dst,
                    src_typestore=src_typestore,
                    dst_typestore=dst_typestore,
                    include_topics=include_topics,
                    exclude_topics=active_exclude_topics,
                )
                return dynamically_excluded_topics
            except RuntimeError as exc:
                details = str(exc)
                failed_msg_type = self._extract_parse_failed_msgtype(details)
                if not failed_msg_type:
                    raise

                if _is_standard_ros_msg_type(failed_msg_type):
                    warn(
                        "rosbags failed to parse standard ROS type %s after sqlite3 rewrite; "
                        "switching to direct rosbag2_py -> rosbag1 fallback.",
                        failed_msg_type,
                    )
                    logger.debug(
                        "Switching from rewritten sqlite3 retry loop to direct rosbag2_py fallback: "
                        "msg_type=%s src=%s include=%s exclude=%s prior_dynamic_excludes=%s",
                        failed_msg_type,
                        _debug_path_state(rewrite_src),
                        include_topics,
                        exclude_topics,
                        dynamically_excluded_topics,
                    )
                    self._cleanup_partial_output(
                        dst,
                        context=f"before direct rosbag2_py fallback for {failed_msg_type}",
                    )
                    self._convert_using_rosbag2_py_writer_fallback(
                        src_path=rewrite_src,
                        dst_path=dst,
                        src_typestore=src_typestore,
                        dst_typestore=dst_typestore,
                        include_topics=include_topics,
                        exclude_topics=exclude_topics,
                    )
                    return []

                offending_topics = self._topics_for_msg_type(
                    rewritten_src,
                    failed_msg_type,
                    include_topics=include_topics,
                    exclude_topics=active_exclude_topics,
                )
                if not offending_topics:
                    raise RuntimeError(
                        f"{details} | Unable to identify topics with parse-failed type {failed_msg_type}"
                    )

                warn(
                    "rosbags failed to parse %s after sqlite3 rewrite; excluding topic(s) and retrying: %s",
                    failed_msg_type,
                    ", ".join(offending_topics),
                )
                logger.debug(
                    "Rewritten sqlite3 parse failure retry: msg_type=%s offending_topics=%s prior_exclude=%s",
                    failed_msg_type,
                    offending_topics,
                    active_exclude_topics,
                )
                next_exclude_topics = list(
                    _merge_topic_lists(active_exclude_topics, offending_topics) or []
                )
                remaining_topics = self._selected_topic_names(
                    rewritten_src,
                    include_topics=include_topics,
                    exclude_topics=next_exclude_topics,
                )
                if not remaining_topics:
                    raise RuntimeError(
                        "All selected topics would be excluded after rewritten sqlite3 parse recovery "
                        f"(latest parse failure: {failed_msg_type}). Aborting instead of producing an empty ROS1 bag."
                    )
                active_exclude_topics = next_exclude_topics
                dynamically_excluded_topics = list(
                    _merge_topic_lists(dynamically_excluded_topics, offending_topics) or []
                )
                active_src = rewritten_src

        raise RuntimeError(
            "Exceeded rewritten sqlite3 parse-recovery attempts while retrying rosbags-convert."
        )

    def _convert_using_rosbags_convert_fallback(
        self,
        src_path: str,
        dst_path: str,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> List[str]:
        logger.debug(
            "Starting rosbags.convert fallback conversion from %s to %s",
            _debug_path_state(src_path),
            _debug_path_state(dst_path),
        )
        try:
            from rosbags.convert import convert as rosbags_convert
        except ImportError as e:
            raise RuntimeError(f"rosbags.convert is required for conversion: {e}")

        temp_parent = Path(dst_path).parent
        temp_parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix=f".rosbags_convert_{Path(src_path).stem}_",
            dir=str(temp_parent),
            ignore_cleanup_errors=True,
        ) as temp_dir:
            convert_input = self._prepare_rosbags_convert_input(src_path, Path(temp_dir))
            logger.debug(
                "rosbags.convert temporary input prepared: src=%s convert_input=%s include=%s exclude=%s",
                _debug_path_state(src_path),
                _debug_path_state(convert_input),
                include_topics,
                exclude_topics,
            )
            try:
                signature = inspect.signature(rosbags_convert)
            except (TypeError, ValueError):
                signature = None
            logger.debug(
                "rosbags.convert callable=%s signature=%s",
                getattr(rosbags_convert, "__qualname__", getattr(rosbags_convert, "__name__", rosbags_convert)),
                signature,
            )

            ros1_dst_storage = "sqlite3"
            ros1_dst_version = 8
            resolved_default_typestore = None
            resolved_dst_typestore = None
            latest_api_attempted = False

            # Newer rosbags versions expose a larger explicit convert() API.
            # Use the function signature to pass named options where available.
            if signature is not None and "dst_storage" in signature.parameters:
                latest_api_attempted = True
                params = signature.parameters

                def _first_param(*names: str) -> Optional[str]:
                    for n in names:
                        if n in params:
                            return n
                    return None

                def _set_if_present(candidates, value):
                    name = _first_param(*candidates)
                    if name is not None:
                        kwargs[name] = value
                        return True
                    return False

                kwargs = {}
                call_args: List[str] = []

                src_key = _first_param("srcs", "src", "sources", "input", "inputs", "source_paths")
                if src_key is not None:
                    kwargs[src_key] = [convert_input]
                else:
                    call_args.append(str(convert_input))

                dst_key = _first_param("dst", "destination", "dst_path", "out", "output")
                if dst_key is not None:
                    kwargs[dst_key] = Path(dst_path)
                else:
                    call_args.append(str(dst_path))

                _set_if_present(("dst_storage",), ros1_dst_storage)
                _set_if_present(("dst_version",), ros1_dst_version)
                _set_if_present(("compress",), False)
                _set_if_present(("compress_mode",), "none")

                try:
                    if "default_typestore" in params:
                        resolved_default_typestore = self._resolve_rosbags_typestore(src_typestore)
                    if "typestore" in params:
                        resolved_dst_typestore = self._resolve_rosbags_typestore(dst_typestore)
                except Exception:
                    logger.debug(
                        "Failed to resolve rosbags typestore objects for latest API path; "
                        "retrying via CLI fallback.",
                        exc_info=True,
                    )
                    return self._convert_using_rosbags_cli_or_rewrite_fallback(
                        src_path=convert_input,
                        dst_path=Path(dst_path),
                        src_typestore=src_typestore,
                        dst_typestore=dst_typestore,
                        include_topics=include_topics,
                        exclude_topics=exclude_topics,
                        temp_root=Path(temp_dir),
                        rewrite_src_path=src_path,
                    )

                if not _set_if_present(
                    ("default_typestore",),
                    resolved_default_typestore if resolved_default_typestore is not None else src_typestore,
                ):
                    _set_if_present(("src_typestore",), src_typestore)
                if not _set_if_present(
                    ("typestore",),
                    resolved_dst_typestore if resolved_dst_typestore is not None else dst_typestore,
                ):
                    _set_if_present(("dst_typestore",), dst_typestore)

                # Topic filtering
                _set_if_present(("include_topics", "include_topic"), include_topics or [])
                _set_if_present(("exclude_topics", "exclude_topic"), exclude_topics or [])
                # Message-type filtering (if API expects it) - keep empty to avoid implicit
                # topic filtering behavior mismatches.
                _set_if_present(("include_msgtypes", "include_msg_types", "include_msgtypes"), [])
                _set_if_present(("exclude_msgtypes", "exclude_msg_types", "exclude_msgtypes"), [])

                try:
                    if call_args:
                        rosbags_convert(*call_args, **kwargs)
                    else:
                        rosbags_convert(**kwargs)
                    return []
                except TypeError as e:
                    self._cleanup_partial_output(dst_path, context=f"latest rosbags.convert TypeError: {e}")
                    logger.debug(
                        "rosbags.convert latest-api call failed: %s", e, exc_info=True
                    )
                except Exception:
                    self._cleanup_partial_output(dst_path, context="latest rosbags.convert failure")
                    logger.debug(
                        "rosbags.convert latest-api call failed; retrying CLI fallback.",
                        exc_info=True,
                    )
                    return self._convert_using_rosbags_cli_or_rewrite_fallback(
                        src_path=convert_input,
                        dst_path=Path(dst_path),
                        src_typestore=src_typestore,
                        dst_typestore=dst_typestore,
                        include_topics=include_topics,
                        exclude_topics=exclude_topics,
                        temp_root=Path(temp_dir),
                        rewrite_src_path=src_path,
                    )

            kwargs = {}
            if latest_api_attempted:
                self._cleanup_partial_output(dst_path, context="retrying alternate rosbags.convert call")
            if include_topics:
                kwargs["include_topics"] = include_topics
            if exclude_topics:
                kwargs["exclude_topics"] = exclude_topics
            if signature is not None and "default_typestore" in signature.parameters:
                try:
                    kwargs["default_typestore"] = (
                        resolved_default_typestore
                        if resolved_default_typestore is not None
                        else self._resolve_rosbags_typestore(src_typestore)
                    )
                except Exception:
                    logger.debug(
                        "Failed to resolve source typestore for rosbags.convert; retrying via CLI fallback.",
                        exc_info=True,
                    )
                    return self._convert_using_rosbags_cli_or_rewrite_fallback(
                        src_path=convert_input,
                        dst_path=Path(dst_path),
                        src_typestore=src_typestore,
                        dst_typestore=dst_typestore,
                        include_topics=include_topics,
                        exclude_topics=exclude_topics,
                        temp_root=Path(temp_dir),
                        rewrite_src_path=src_path,
                    )
            elif signature is not None and "src_typestore" in signature.parameters:
                kwargs["src_typestore"] = src_typestore
            if signature is not None and "typestore" in signature.parameters:
                try:
                    kwargs["typestore"] = (
                        resolved_dst_typestore
                        if resolved_dst_typestore is not None
                        else self._resolve_rosbags_typestore(dst_typestore)
                    )
                except Exception:
                    logger.debug(
                        "Failed to resolve destination typestore for rosbags.convert; retrying via CLI fallback.",
                        exc_info=True,
                    )
                    return self._convert_using_rosbags_cli_or_rewrite_fallback(
                        src_path=convert_input,
                        dst_path=Path(dst_path),
                        src_typestore=src_typestore,
                        dst_typestore=dst_typestore,
                        include_topics=include_topics,
                        exclude_topics=exclude_topics,
                        temp_root=Path(temp_dir),
                        rewrite_src_path=src_path,
                    )
            elif signature is not None and "dst_typestore" in signature.parameters:
                kwargs["dst_typestore"] = dst_typestore

            # Older/alternate conversion helpers
            try:
                rosbags_convert(convert_input, Path(dst_path), **kwargs)
                return []
            except KeyError as e:
                self._cleanup_partial_output(dst_path, context=f"legacy rosbags.convert KeyError: {e}")
                warn(
                    f"rosbags-convert conversion API failed with KeyError({e}); "
                    "retrying with rosbags-convert CLI fallback."
                )
                return self._convert_using_rosbags_cli_or_rewrite_fallback(
                    src_path=convert_input,
                    dst_path=Path(dst_path),
                    src_typestore=src_typestore,
                    dst_typestore=dst_typestore,
                    include_topics=include_topics,
                    exclude_topics=exclude_topics,
                    temp_root=Path(temp_dir),
                    rewrite_src_path=src_path,
                )
            except TypeError as e:
                if "required positional arguments" in str(e):
                    warn(
                        "Installed rosbags.convert requires the newer API signature; "
                        "retrying fallback via rosbags-convert CLI."
                    )
                    self._cleanup_partial_output(dst_path, context=f"legacy rosbags.convert TypeError: {e}")
                    return self._convert_using_rosbags_cli_or_rewrite_fallback(
                        src_path=convert_input,
                        dst_path=Path(dst_path),
                        src_typestore=src_typestore,
                        dst_typestore=dst_typestore,
                        include_topics=include_topics,
                        exclude_topics=exclude_topics,
                        temp_root=Path(temp_dir),
                        rewrite_src_path=src_path,
                    )
                if include_topics and "include_topics" in str(e):
                    warn(
                        "Installed rosbags.convert does not support include_topics; "
                        "retrying fallback without that filter."
                    )
                    self._cleanup_partial_output(dst_path, context="legacy rosbags.convert include_topics retry")
                    kwargs.pop("include_topics", None)
                    rosbags_convert(convert_input, Path(dst_path), **kwargs)
                    return []
                raise

    def _convert_using_rosbags_cli_fallback(
        self,
        src_path: Path,
        dst_path: Path,
        src_typestore: str = "ros2_jazzy",
        dst_typestore: str = "ros1_noetic",
        include_topics: Optional[List[str]] = None,
        exclude_topics: Optional[List[str]] = None,
    ) -> None:
        self._cleanup_partial_output(dst_path, context="rosbags-convert CLI fallback preflight")
        cmd = ["rosbags-convert", "--src", str(src_path), "--dst", str(dst_path)]
        if src_typestore:
            cmd.extend(["--src-typestore", src_typestore])
        if dst_typestore:
            cmd.extend(["--dst-typestore", dst_typestore])
        if include_topics:
            cmd.extend(["--include-topic", *include_topics])
        if exclude_topics:
            cmd.extend(["--exclude-topic", *exclude_topics])

        logger.debug(
            "Running rosbags-convert CLI fallback: src=%s dst=%s src_typestore=%s dst_typestore=%s include=%s exclude=%s",
            _debug_path_state(src_path),
            _debug_path_state(dst_path),
            src_typestore,
            dst_typestore,
            include_topics,
            exclude_topics,
        )
        self._debug_log_ros2_bag_summary(src_path, "rosbags-convert source summary")
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
        )
        self._debug_log_subprocess_result("rosbags-convert CLI fallback", cmd, result)
        if result.returncode == 0:
            logger.debug("rosbags-convert created %s", _debug_path_state(dst_path))
            return

        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        details = stderr or stdout or f"exit code {result.returncode}"
        if "KeyError" in details:
            raise RuntimeError(f"rosbags-convert CLI reported schema KeyError: {details}")
        raise RuntimeError(f"rosbags-convert CLI fallback failed: {details}")

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
        """
        Rewrite a ROS 1 bag with renamed topics.  Produces a new file with
        the '_remapped' suffix so the original is never overwritten.
        """
        try:
            import rosbag  # noqa: F401 — only available in a ROS 1 environment
        except ImportError:
            warn(
                "Topic remapping requires 'rosbag' (ROS 1).  "
                "Run this step inside a ROS 1 environment."
            )
            return bag_path

        out = bag_path.parent / (bag_path.stem + "_remapped.bag")
        with rosbag.Bag(str(out), "w") as out_bag, rosbag.Bag(str(bag_path), "r") as in_bag:
            for topic, msg, t in in_bag.read_messages():
                out_topic = remap.get(topic, topic)
                out_bag.write(out_topic, msg, t)
        log(f"Topics remapped → {out}")
        return out

    # ---------- folder batch conversion ----------
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
