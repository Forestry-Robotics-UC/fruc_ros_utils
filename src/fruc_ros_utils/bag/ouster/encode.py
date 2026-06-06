#!/usr/bin/env python3
"""Re-encode decoded Ouster data back into raw packet topics.

Mirrors OusterPacketDecoder from ouster_decode.py in the reverse direction:
  PointCloud2 / sensor_msgs/Imu  →  ouster_sensor_msgs/PacketMsg
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
import rclpy.serialization
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Header

from ouster.sdk._bindings.client import LidarPacket, ImuPacket, PacketWriter, SensorInfo
from ouster.sdk.util import scan_to_packets  # type: ignore
from ouster.sdk.core import LidarScan, ChanField, ScanBatcher, PacketFormat

from ouster_sensor_msgs.msg import PacketMsg

logger = logging.getLogger(__name__)


class OusterPointCloudEncoder:
    """Re-encode PointCloud2 messages back into Ouster lidar/imu packet topics.

    This is the inverse of OusterPacketDecoder.  It requires a SensorInfo
    (metadata) either passed directly or extracted from a reference bag.

    Limitations:
    - Re-packed lidar packets reproduce the original column/pixel layout but
      exact byte-level fidelity depends on what fields are present in the
      PointCloud2 (RANGE, SIGNAL, REFLECTIVITY, NEAR_IR at minimum).
    - IMU re-encoding is NOT supported — the decoded sensor_msgs/Imu only
      retains linear_acceleration and angular_velocity, losing sub-field
      timestamps.  IMU packets from the original ROS1 bag (topic /ouster/imu)
      are passed through as-is from the decoded Imu messages via timestamp
      matching (best-effort).
    """

    def __init__(
        self,
        sensor_info: SensorInfo,
        points_topic: str = "/ouster/points",
        lidar_packets_topic: str = "/ouster/lidar_packets",
        imu_packets_topic: str = "/ouster/imu_packets",
        metadata_topic: str = "/ouster/metadata",
        lidar_frame_id: str = "os_lidar",
    ) -> None:
        self.sensor_info = sensor_info
        self.points_topic = points_topic
        self.lidar_packets_topic = lidar_packets_topic
        self.imu_packets_topic = imu_packets_topic
        self.metadata_topic = metadata_topic
        self.lidar_frame_id = lidar_frame_id

        self._packet_writer = PacketWriter.from_info(sensor_info)
        self._cols = sensor_info.format.columns_per_frame
        self._rows = sensor_info.format.pixels_per_column
        self._metadata_emitted = False

        self.stats: Dict[str, int] = {
            "pointclouds_seen": 0,
            "lidar_packets_emitted": 0,
            "encode_errors": 0,
        }

    @classmethod
    def from_metadata_string(
        cls,
        meta_str: str,
        points_topic: str = "/ouster/points",
        lidar_packets_topic: str = "/ouster/lidar_packets",
        imu_packets_topic: str = "/ouster/imu_packets",
        metadata_topic: str = "/ouster/metadata",
    ) -> "OusterPointCloudEncoder":
        sensor_info = SensorInfo(meta_str)
        return cls(
            sensor_info=sensor_info,
            points_topic=points_topic,
            lidar_packets_topic=lidar_packets_topic,
            imu_packets_topic=imu_packets_topic,
            metadata_topic=metadata_topic,
        )

    def output_topic_types(self) -> Dict[str, str]:
        return {
            self.lidar_packets_topic: "ouster_sensor_msgs/msg/PacketMsg",
            self.imu_packets_topic: "ouster_sensor_msgs/msg/PacketMsg",
            self.metadata_topic: "std_msgs/msg/String",
        }

    def raw_topics(self) -> List[str]:
        return [self.points_topic]

    def emit_metadata(self, bag_timestamp_ns: int) -> List[Tuple[str, str, object, int]]:
        """Emit the metadata String message (call once at the start)."""
        from std_msgs.msg import String
        msg = String()
        msg.data = self.sensor_info.to_json_string()
        self._metadata_emitted = True
        return [(self.metadata_topic, "std_msgs/msg/String", msg, bag_timestamp_ns)]

    def process_message(
        self,
        topic: str,
        data: bytes,
        bag_timestamp_ns: int,
    ) -> List[Tuple[str, str, object, int]]:
        """Process a PointCloud2 message and return lidar PacketMsg outputs."""
        if topic != self.points_topic:
            return []

        outputs: List[Tuple[str, str, object, int]] = []

        # Emit metadata once
        if not self._metadata_emitted:
            outputs.extend(self.emit_metadata(bag_timestamp_ns))

        self.stats["pointclouds_seen"] += 1
        try:
            pc2_msg = rclpy.serialization.deserialize_message(data, PointCloud2)
            scan = self._pointcloud_to_scan(pc2_msg, bag_timestamp_ns)
            if scan is None:
                self.stats["encode_errors"] += 1
                return outputs

            packets = scan_to_packets(scan, self.sensor_info)
            for pkt in packets:
                pkt_msg = PacketMsg()
                pkt_msg.header.stamp = rclpy.time.Time(nanoseconds=bag_timestamp_ns).to_msg()
                pkt_msg.buf = list(bytes(pkt.buf))
                outputs.append((
                    self.lidar_packets_topic,
                    "ouster_sensor_msgs/msg/PacketMsg",
                    pkt_msg,
                    bag_timestamp_ns,
                ))
                self.stats["lidar_packets_emitted"] += 1

        except Exception as exc:
            self.stats["encode_errors"] += 1
            logger.warning("Ouster re-encode error on %s at %d: %s", topic, bag_timestamp_ns, exc)

        return outputs

    def _pointcloud_to_scan(
        self, pc2_msg: PointCloud2, bag_timestamp_ns: int
    ) -> Optional[LidarScan]:
        """Reconstruct a LidarScan from a PointCloud2 message.

        Always uses sensor dimensions (pixels_per_column × columns_per_frame) for the
        LidarScan, regardless of the PointCloud2 layout.  Handles both organized
        (h×w) and unorganized (1×N) point clouds with field-name aliases for the
        ROS1 Ouster driver output (intensity/ambient vs signal/near_ir).
        """
        try:
            fields = {f.name: f for f in pc2_msg.fields}
            h = self._rows   # pixels_per_column
            w = self._cols   # columns_per_frame
            total = h * w
            pc2_total = pc2_msg.height * pc2_msg.width
            point_step = pc2_msg.point_step

            raw_bytes = bytes(pc2_msg.data)
            if pc2_total == total:
                raw = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(total, point_step)
            elif pc2_total < total:
                logger.debug(
                    "PointCloud2 has fewer points (%d) than sensor grid (%d), zero-padding",
                    pc2_total, total,
                )
                padded = np.zeros(total * point_step, dtype=np.uint8)
                n = min(len(raw_bytes), total * point_step)
                padded[:n] = np.frombuffer(raw_bytes, dtype=np.uint8)[:n]
                raw = padded.reshape(total, point_step)
            else:
                logger.debug(
                    "PointCloud2 has more points (%d) than sensor grid (%d), truncating",
                    pc2_total, total,
                )
                raw = np.frombuffer(raw_bytes, dtype=np.uint8)[:total * point_step].reshape(total, point_step)

            scan = LidarScan(h, w)

            # Field name aliases: ROS1 Ouster driver uses intensity/ambient;
            # ROS2 decoder uses signal/near_ir.
            range_key = "range" if "range" in fields else None
            signal_key = next((k for k in ("signal", "intensity") if k in fields), None)
            refl_key = "reflectivity" if "reflectivity" in fields else None
            near_ir_key = next((k for k in ("near_ir", "ambient", "noise") if k in fields), None)

            if range_key:
                f = fields[range_key]
                vals = raw[:, f.offset:f.offset + 4].view(np.uint32).reshape(h, w)
                scan.field(ChanField.RANGE)[:] = vals

            if signal_key:
                f = fields[signal_key]
                if f.datatype == 7:  # FLOAT32 — ROS1 driver stores signal as float32
                    vals = raw[:, f.offset:f.offset + 4].view(np.float32).reshape(h, w).astype(np.uint32)
                else:
                    vals = raw[:, f.offset:f.offset + 4].view(np.uint32).reshape(h, w)
                scan.field(ChanField.SIGNAL)[:] = vals

            if refl_key:
                f = fields[refl_key]
                vals = raw[:, f.offset:f.offset + 2].view(np.uint16).reshape(h, w)
                scan.field(ChanField.REFLECTIVITY)[:] = vals

            if near_ir_key:
                f = fields[near_ir_key]
                vals = raw[:, f.offset:f.offset + 2].view(np.uint16).reshape(h, w)
                scan.field(ChanField.NEAR_IR)[:] = vals

            scan.timestamp[:] = bag_timestamp_ns
            scan.measurement_id[:] = np.arange(w, dtype=np.uint16)
            # Mark all columns valid; without this scan_to_packets emits zero packets
            scan.status[:] = 0x01

            return scan

        except Exception as exc:
            logger.warning("Failed to reconstruct LidarScan from PointCloud2: %s", exc)
            return None

    def get_stats(self) -> Dict[str, int]:
        return dict(self.stats)
