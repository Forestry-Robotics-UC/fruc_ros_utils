#!/usr/bin/env python3
"""Reusable Ouster packet decoding helpers for offline bag conversion."""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
    from std_msgs.msg import Header, String
except ModuleNotFoundError as _err:
    raise ModuleNotFoundError(
        "Ouster packet decoding requires the ROS2 Python package 'rclpy'. "
        "This command is not available in the ROS1-only (Noetic) container."
    ) from _err

from ouster.sdk._bindings.client import ImuPacket, LidarPacket
from ouster.sdk.core import (
    ChanField,
    LidarScan,
    PacketFormat,
    ScanBatcher,
    SensorInfo,
    XYZLut,
    destagger,
)
from ouster_sensor_msgs.msg import PacketMsg

logger = logging.getLogger(__name__)


class OusterPacketDecoder:
    """Decode Ouster packet topics into standard ROS2 sensor messages."""

    def __init__(
        self,
        metadata_topic: str = "/ouster/metadata",
        lidar_packets_topic: str = "/ouster/lidar_packets",
        imu_packets_topic: str = "/ouster/imu_packets",
        points_topic: str = "/ouster/points",
        depth_topic: str = "/ouster/depth_image",
        imu_topic: str = "/ouster/imu",
        lidar_frame_id: str = "os_lidar",
        imu_frame_id: str = "os_imu",
        output_mode: str = "points",
        destagger_output: bool = False,
        metadata_file: Optional[str] = None,
        input_bag_path: Optional[str] = None,
    ) -> None:
        self.metadata_topic = metadata_topic
        self.lidar_packets_topic = lidar_packets_topic
        self.imu_packets_topic = imu_packets_topic
        self.points_topic = points_topic
        self.depth_topic = depth_topic
        self.imu_topic = imu_topic
        self.lidar_frame_id = lidar_frame_id
        self.imu_frame_id = imu_frame_id
        self.output_mode = output_mode
        self.destagger_output = destagger_output
        self.metadata_file = metadata_file
        self.input_bag_path = input_bag_path

        self.sensor_info: Optional[SensorInfo] = None
        self.xyzlut: Optional[XYZLut] = None
        self.packet_format: Optional[PacketFormat] = None
        self.scan_batcher: Optional[ScanBatcher] = None
        self.lidar_scan: Optional[LidarScan] = None

        self.has_signal = False
        self.has_reflectivity = False
        self.has_range = False
        self.has_ambient = False

        self.first_packet_timestamp: Optional[int] = None
        self.first_column_timestamp_ref: Optional[int] = None
        self.stats = {
            "metadata_messages": 0,
            "metadata_fallback_used": 0,
            "lidar_packets_seen": 0,
            "imu_packets_seen": 0,
            "scans_emitted": 0,
            "depth_images_emitted": 0,
            "imu_messages_emitted": 0,
            "messages_skipped": 0,
            "decode_errors": 0,
        }

    def initialize_metadata_fallback_if_available(self) -> bool:
        """Initialize decoder state from an explicit metadata source when possible.

        The fallback search order is:
        1. ``metadata_file`` passed by the caller
        2. sibling split-bag member ``*_0.mcap`` when the current input is ``*_N.mcap``

        Metadata may come from a plain text file or from a rosbag2 source that carries
        ``self.metadata_topic`` exactly once.
        """
        if self.sensor_info is not None:
            return False
        candidate_paths: List[Path] = []
        if self.metadata_file:
            candidate_paths.append(Path(self.metadata_file))
        derived = self._derive_split_metadata_bag()
        if derived is not None:
            candidate_paths.append(derived)

        for metadata_source in candidate_paths:
            payload = self._load_metadata_payload(metadata_source)
            if not payload:
                continue
            self.initialize_sensor_from_string(payload)
            self.stats["metadata_fallback_used"] += 1
            logger.info("Initialized Ouster metadata fallback from %s", metadata_source)
            return True
        return False

    def initialize_sensor_from_string(self, meta_str: str) -> None:
        self.sensor_info = SensorInfo(meta_str)
        self.xyzlut = XYZLut(self.sensor_info)
        self.packet_format = PacketFormat(self.sensor_info)
        self.scan_batcher = ScanBatcher(self.sensor_info)

        width = self.sensor_info.format.columns_per_frame
        height = self.sensor_info.format.pixels_per_column
        self.lidar_scan = LidarScan(height, width)

        self.has_signal = self._scan_has_field(ChanField.SIGNAL)
        self.has_reflectivity = self._scan_has_field(ChanField.REFLECTIVITY)
        self.has_range = self._scan_has_field(ChanField.RANGE)
        self.has_ambient = self._scan_has_field(ChanField.NEAR_IR)

    def output_topic_types(self) -> Dict[str, str]:
        topics = {self.imu_topic: "sensor_msgs/msg/Imu"}
        if self.output_mode in ("points", "both"):
            topics[self.points_topic] = "sensor_msgs/msg/PointCloud2"
        if self.output_mode in ("depth", "both"):
            topics[self.depth_topic] = "sensor_msgs/msg/Image"
        return topics

    def raw_topics(self) -> List[str]:
        return [self.metadata_topic, self.lidar_packets_topic, self.imu_packets_topic]

    def process_message(
        self,
        topic: str,
        data: bytes,
        bag_timestamp_ns: int,
    ) -> List[Tuple[str, str, object, int]]:
        outputs: List[Tuple[str, str, object, int]] = []

        if topic == self.metadata_topic:
            meta_msg = deserialize_message(data, String)
            self.initialize_sensor_from_string(meta_msg.data)
            self.stats["metadata_messages"] += 1
            return outputs

        if self.sensor_info is None:
            self.stats["messages_skipped"] += 1
            return outputs

        if topic == self.imu_packets_topic:
            self.stats["imu_packets_seen"] += 1
            imu_msg = self._decode_imu_message(data, bag_timestamp_ns)
            if imu_msg is not None:
                outputs.append((self.imu_topic, "sensor_msgs/msg/Imu", imu_msg, bag_timestamp_ns))
                self.stats["imu_messages_emitted"] += 1
            else:
                self.stats["messages_skipped"] += 1
            return outputs

        if topic != self.lidar_packets_topic:
            return outputs

        self.stats["lidar_packets_seen"] += 1
        scan_outputs = self._decode_lidar_packet(data, bag_timestamp_ns)
        outputs.extend(scan_outputs)
        if not scan_outputs:
            self.stats["messages_skipped"] += 1
        return outputs

    def _decode_imu_message(self, data: bytes, bag_timestamp_ns: int) -> Optional[Imu]:
        if self.packet_format is None:
            logger.warning("Skipping Ouster IMU packet because packet_format is not initialized.")
            return None
        msg_stamp = self.extract_packet_timestamp(data) or bag_timestamp_ns
        raw_buf = self.get_raw_buffer(data)
        try:
            imu_pkt = ImuPacket(self.packet_format.imu_packet_size)
            np.frombuffer(imu_pkt.buf, dtype=np.uint8)[:] = np.frombuffer(raw_buf, dtype=np.uint8)

            imu_msg = Imu()
            imu_msg.header.stamp = rclpy.time.Time(nanoseconds=int(msg_stamp)).to_msg()
            imu_msg.header.frame_id = self.imu_frame_id
            imu_msg.linear_acceleration.x = self.packet_format.imu_la_x(imu_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.y = self.packet_format.imu_la_y(imu_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.z = self.packet_format.imu_la_z(imu_pkt.buf) * 9.80665
            imu_msg.angular_velocity.x = math.radians(self.packet_format.imu_av_x(imu_pkt.buf))
            imu_msg.angular_velocity.y = math.radians(self.packet_format.imu_av_y(imu_pkt.buf))
            imu_msg.angular_velocity.z = math.radians(self.packet_format.imu_av_z(imu_pkt.buf))
            return imu_msg
        except Exception as exc:
            self.stats["decode_errors"] += 1
            logger.warning("Failed to decode Ouster IMU packet: %s", exc)
            return None

    def _decode_lidar_packet(
        self,
        data: bytes,
        bag_timestamp_ns: int,
    ) -> List[Tuple[str, str, object, int]]:
        if any(
            current is None
            for current in (
                self.packet_format,
                self.scan_batcher,
                self.lidar_scan,
                self.xyzlut,
                self.sensor_info,
            )
        ):
            logger.warning("Skipping Ouster lidar packet because decoder state is not initialized.")
            return []

        outputs: List[Tuple[str, str, object, int]] = []
        msg_stamp = self.extract_packet_timestamp(data) or bag_timestamp_ns

        raw_buf = self.get_raw_buffer(data)
        try:
            lidar_pkt = LidarPacket(self.packet_format.lidar_packet_size)
            dst = np.frombuffer(lidar_pkt.buf, dtype=np.uint8)
            src = np.frombuffer(raw_buf, dtype=np.uint8)
            dst[: len(src)] = src[: len(dst)]
        except Exception as exc:
            self.stats["decode_errors"] += 1
            logger.warning("Failed to copy raw Ouster lidar packet bytes: %s", exc)
            return []

        if self.first_packet_timestamp is None:
            self.first_packet_timestamp = int(msg_stamp)

        try:
            completed = self.scan_batcher(lidar_pkt, self.lidar_scan)
        except Exception as exc:
            self.stats["decode_errors"] += 1
            logger.warning("Ouster scan batcher failed on lidar packet: %s", exc)
            return []
        if not completed:
            return outputs

        header = Header()
        header.stamp = rclpy.time.Time(nanoseconds=int(self.first_packet_timestamp)).to_msg()
        header.frame_id = self.lidar_frame_id

        if self.output_mode in ("points", "both"):
            pointcloud_msg = self._build_pointcloud_message(header)
            if pointcloud_msg is not None:
                outputs.append(
                    (self.points_topic, "sensor_msgs/msg/PointCloud2", pointcloud_msg, bag_timestamp_ns)
                )
                self.stats["scans_emitted"] += 1

        if self.output_mode in ("depth", "both"):
            depth_msg = self._build_depth_message(header)
            if depth_msg is not None:
                outputs.append((self.depth_topic, "sensor_msgs/msg/Image", depth_msg, bag_timestamp_ns))
                self.stats["depth_images_emitted"] += 1

        self.first_packet_timestamp = None
        self.first_column_timestamp_ref = None
        return outputs

    @staticmethod
    def extract_packet_timestamp(data: bytes) -> int:
        try:
            pkt_msg = deserialize_message(data, PacketMsg)
            stamp = pkt_msg.header.stamp
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except Exception:
            return 0

    @staticmethod
    def get_raw_buffer(data: bytes) -> bytes:
        try:
            pkt_msg = deserialize_message(data, PacketMsg)
            return bytes(pkt_msg.buf)
        except Exception:
            return bytes(data)

    def _scan_has_field(self, chan_field) -> bool:
        try:
            if self.lidar_scan is None:
                return False
            self.lidar_scan.field(chan_field)
            return True
        except Exception:
            return False

    def _build_depth_message(self, header: Header) -> Optional[Image]:
        if self.lidar_scan is None:
            logger.warning("Cannot build depth message because lidar_scan is not initialized.")
            return None
        try:
            depth_mm = self.lidar_scan.field(ChanField.RANGE)
        except Exception as exc:
            self.stats["decode_errors"] += 1
            logger.warning("Failed to read RANGE channel for depth image: %s", exc)
            return None
        if self.destagger_output:
            if self.sensor_info is None:
                logger.warning("Cannot destagger depth image because sensor_info is not initialized.")
                return None
            depth_mm = destagger(self.sensor_info, depth_mm)
        depth_m = np.ascontiguousarray(depth_mm.astype(np.float32) * 1e-3)

        msg = Image()
        msg.header = header
        msg.height = int(depth_m.shape[0])
        msg.width = int(depth_m.shape[1])
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = int(depth_m.shape[1] * 4)
        msg.data = depth_m.tobytes()
        return msg

    def _build_pointcloud_message(self, header: Header) -> Optional[PointCloud2]:
        """Convert the current completed Ouster scan into a PointCloud2 message."""
        if self.lidar_scan is None or self.xyzlut is None or self.sensor_info is None:
            logger.warning("Cannot build point cloud because decoder state is incomplete.")
            return None

        try:
            xyz = self.xyzlut(self.lidar_scan)
            points = xyz.reshape(-1, 3)
        except Exception as exc:
            self.stats["decode_errors"] += 1
            logger.warning("Failed to compute XYZ lookup for Ouster scan: %s", exc)
            return None
        points = points * np.array([-1.0, -1.0, 1.0], dtype=np.float32)
        mask = np.any(points != 0, axis=1)
        valid_points = points[mask]
        if valid_points.size == 0:
            return None

        additional_fields: Dict[str, np.ndarray] = {}
        height = self.sensor_info.format.pixels_per_column
        width = self.sensor_info.format.columns_per_frame

        column_timestamps_ns = self.lidar_scan.timestamp.astype(np.int64)
        if self.first_column_timestamp_ref is None:
            self.first_column_timestamp_ref = int(column_timestamps_ns[0]) if len(column_timestamps_ns) else 0
        relative_timestamps = (column_timestamps_ns - np.int64(self.first_column_timestamp_ref)).astype(np.uint32)
        full_timestamp_field = np.tile(relative_timestamps, (height, 1))
        additional_fields["t"] = full_timestamp_field.reshape(-1)[mask]

        ring_data = np.repeat(np.arange(height, dtype=np.uint16), width)
        additional_fields["ring"] = ring_data[mask].astype(np.uint16)

        if self.has_signal:
            try:
                signal = self.lidar_scan.field(ChanField.SIGNAL)
                additional_fields["intensity"] = signal.reshape(-1)[mask].astype(np.float32)
            except Exception as exc:
                logger.warning("Failed to extract SIGNAL field: %s", exc)
        if self.has_reflectivity:
            try:
                reflectivity = self.lidar_scan.field(ChanField.REFLECTIVITY)
                additional_fields["reflectivity"] = reflectivity.reshape(-1)[mask].astype(np.uint16)
            except Exception as exc:
                logger.warning("Failed to extract REFLECTIVITY field: %s", exc)
        if self.has_ambient:
            try:
                near_ir = self.lidar_scan.field(ChanField.NEAR_IR)
                additional_fields["ambient"] = near_ir.reshape(-1)[mask].astype(np.uint16)
            except Exception as exc:
                logger.warning("Failed to extract NEAR_IR field: %s", exc)
        if self.has_range:
            try:
                range_data = self.lidar_scan.field(ChanField.RANGE)
                additional_fields["range"] = range_data.reshape(-1)[mask].astype(np.uint32)
            except Exception as exc:
                logger.warning("Failed to extract RANGE field for point cloud: %s", exc)

        return self._create_cloud_with_fields(header, valid_points, additional_fields)

    def _load_metadata_payload(self, source: Path) -> Optional[str]:
        """Load an Ouster metadata payload from a text file or rosbag2 source."""
        if not source.exists():
            return None
        if source.is_file() and source.suffix.lower() in {".json", ".txt"}:
            return source.read_text(encoding="utf-8")
        if source.is_file() and source.suffix.lower() in {".mcap", ".db3"}:
            return self._extract_metadata_from_bag(source)
        if source.is_dir():
            return self._extract_metadata_from_bag(source)
        return None

    def _extract_metadata_from_bag(self, bag_path: Path) -> Optional[str]:
        """Extract the first matching metadata payload from a rosbag2 source."""
        try:
            from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        except ImportError as exc:
            logger.warning("rosbag2_py is required to load Ouster metadata from %s: %s", bag_path, exc)
            return None

        storage_id = ""
        if bag_path.is_file():
            storage_id = {".mcap": "mcap", ".db3": "sqlite3"}.get(bag_path.suffix.lower(), "")
        reader = SequentialReader()
        try:
            reader.open(
                StorageOptions(uri=str(bag_path), storage_id=storage_id),
                ConverterOptions("", ""),
            )
            while reader.has_next():
                topic_name, data, _timestamp = reader.read_next()
                if topic_name != self.metadata_topic:
                    continue
                meta_msg = deserialize_message(data, String)
                return meta_msg.data
        except Exception as exc:
            logger.warning("Failed to extract Ouster metadata from %s: %s", bag_path, exc)
            return None
        return None

    def _derive_split_metadata_bag(self) -> Optional[Path]:
        """Return the sibling ``*_0.mcap`` split member when converting ``*_N.mcap``."""
        if not self.input_bag_path:
            return None
        input_path = Path(self.input_bag_path).resolve()
        candidate_names: List[Path] = []
        if input_path.is_file():
            stem = input_path.name
            if stem.endswith(".mcap"):
                stem = stem[:-5]
            if "_" in stem:
                prefix, suffix = stem.rsplit("_", 1)
                if suffix.isdigit() and suffix != "0":
                    candidate_names.append(input_path.with_name(f"{prefix}_0.mcap"))
        for candidate in candidate_names:
            if candidate.exists():
                return candidate
        return None

    def get_stats(self) -> Dict[str, int]:
        return dict(self.stats)

    @staticmethod
    def _create_cloud_with_fields(
        header: Header,
        points: np.ndarray,
        additional_fields: Dict[str, np.ndarray],
    ) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="t", offset=20, datatype=PointField.UINT32, count=1),
            PointField(name="reflectivity", offset=24, datatype=PointField.UINT16, count=1),
            PointField(name="ring", offset=26, datatype=PointField.UINT16, count=1),
            PointField(name="ambient", offset=28, datatype=PointField.UINT16, count=1),
            PointField(name="range", offset=32, datatype=PointField.UINT32, count=1),
        ]

        dtype_list = [
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("pad1", np.uint32),
            ("intensity", np.float32),
            ("t", np.uint32),
            ("reflectivity", np.uint16),
            ("ring", np.uint16),
            ("ambient", np.uint16),
            ("pad2", np.uint16),
            ("range", np.uint32),
            ("pad3", np.uint32),
            ("pad4", np.uint32),
            ("pad5", np.uint32),
        ]

        cloud_data = np.zeros(len(points), dtype=dtype_list)
        cloud_data["x"] = points[:, 0]
        cloud_data["y"] = points[:, 1]
        cloud_data["z"] = points[:, 2]

        for field_name in ("intensity", "t", "reflectivity", "ring", "ambient", "range"):
            if field_name in additional_fields:
                cloud_data[field_name] = additional_fields[field_name]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 48
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = cloud_data.tobytes()
        return msg
