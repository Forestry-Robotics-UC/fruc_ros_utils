#!/usr/bin/env python3
"""Decode ffmpeg_image_transport packet topics into sensor_msgs/Image."""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from sensor_msgs.msg import Image

from rosbags.typesys import Stores, get_typestore, get_types_from_msg

logger = logging.getLogger(__name__)

_FFMPEG_PACKET_MSG_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"
_FFMPEG_PACKET_MSG_DEF = """\
std_msgs/Header header
int32 width
int32 height
string encoding
uint64 pts
uint8 flags
bool is_bigendian
uint8[] data
"""

_ROS_IMAGE_TO_AV_FORMAT = {
    "bgr8": "bgr24",
    "rgb8": "rgb24",
    "mono8": "gray",
}


def _parse_packet_encoding(encoding: str) -> Tuple[str, str]:
    """Return (codec_name, ros_image_encoding) parsed from packet encoding metadata."""
    tokens = [part.strip().lower() for part in str(encoding).split(";") if part.strip()]
    codec_name = tokens[0] if tokens else "h264"

    preferred_ros_encoding = "bgr8"
    for token in tokens[1:]:
        if token in ("bgr8", "rgb8", "mono8"):
            preferred_ros_encoding = token
            break
        if token == "bgr24":
            preferred_ros_encoding = "bgr8"
            break
        if token == "rgb24":
            preferred_ros_encoding = "rgb8"
            break

    return codec_name, preferred_ros_encoding


class FFMPEGPacketDecoder:
    """Decode ffmpeg packet topics into Image topics."""

    def __init__(self, topic_map: Dict[str, str]) -> None:
        if not topic_map:
            raise ValueError("FFMPEGPacketDecoder requires a non-empty topic_map.")
        self.topic_map = dict(topic_map)

        self._typestore = get_typestore(Stores.ROS2_JAZZY)
        self._typestore.register(
            get_types_from_msg(_FFMPEG_PACKET_MSG_DEF, _FFMPEG_PACKET_MSG_TYPE)
        )

        try:
            import av  # type: ignore
        except Exception as exc:  # noqa: BLE001
            raise ModuleNotFoundError(
                "FFmpeg decode requires Python package 'av' (PyAV). "
                "Install python3-av in the runtime image/environment."
            ) from exc
        self._av = av
        try:
            self._av.logging.set_level(self._av.logging.ERROR)
        except Exception:
            logger.debug("Could not configure PyAV logging level", exc_info=True)

        self._codec_contexts: Dict[str, object] = {}
        self._codec_names: Dict[str, str] = {}
        self._ros_encodings: Dict[str, str] = {}
        self._synced_on_keyframe: Dict[str, bool] = {}
        self._decode_warning_counts: Dict[str, int] = {}
        self.stats = {
            "packets_seen": 0,
            "frames_emitted": 0,
            "packets_without_frames": 0,
            "decode_errors": 0,
        }

    def output_topic_types(self) -> Dict[str, str]:
        return {dst_topic: "sensor_msgs/msg/Image" for dst_topic in self.topic_map.values()}

    def raw_topics(self) -> List[str]:
        return sorted(self.topic_map.keys())

    def process_message(
        self,
        topic: str,
        data: bytes,
        bag_timestamp_ns: int,
    ) -> List[Tuple[str, str, object, int]]:
        dst_topic = self.topic_map.get(topic)
        if dst_topic is None:
            return []

        self.stats["packets_seen"] += 1
        try:
            packet_msg = self._typestore.deserialize_cdr(data, _FFMPEG_PACKET_MSG_TYPE)
        except Exception as exc:
            self.stats["decode_errors"] += 1
            self._warn_decode_failure(topic, f"Failed to deserialize FFmpeg packet: {exc}")
            return []

        codec_name, desired_encoding = _parse_packet_encoding(getattr(packet_msg, "encoding", ""))
        flags = int(getattr(packet_msg, "flags", 0) or 0)
        is_keyframe = bool(flags & 0x01)
        codec_ctx = self._codec_contexts.get(topic)
        if codec_ctx is None or self._codec_names.get(topic) != codec_name:
            codec_ctx = self._reset_codec_context(topic, codec_name, desired_encoding)
            if codec_ctx is None:
                self.stats["decode_errors"] += 1
                self._warn_decode_failure(
                    topic,
                    f"Failed to create FFmpeg decoder context for codec {codec_name}",
                )
                return []

        # Split bag members can begin mid-stream; wait for a keyframe before decoding.
        if not self._synced_on_keyframe.get(topic, False):
            if not is_keyframe:
                self.stats["packets_without_frames"] += 1
                return []
            self._synced_on_keyframe[topic] = True

        try:
            payload = packet_msg.data
            payload_bytes = payload.tobytes() if hasattr(payload, "tobytes") else bytes(payload)
            av_packet = self._av.Packet(payload_bytes)
            pts = int(getattr(packet_msg, "pts", 0) or 0)
            if pts:
                av_packet.pts = pts
            frames = codec_ctx.decode(av_packet)
        except Exception as exc:
            self.stats["decode_errors"] += 1
            # On keyframe decode failures, reset context and retry once.
            if is_keyframe:
                codec_ctx = self._reset_codec_context(topic, codec_name, desired_encoding)
                if codec_ctx is not None:
                    try:
                        self._synced_on_keyframe[topic] = True
                        retry_packet = self._av.Packet(payload_bytes)
                        if pts:
                            retry_packet.pts = pts
                        frames = codec_ctx.decode(retry_packet)
                    except Exception as retry_exc:
                        self._warn_decode_failure(
                            topic,
                            f"Failed to decode FFmpeg keyframe payload: {retry_exc}",
                        )
                        return []
                else:
                    self._warn_decode_failure(
                        topic,
                        f"Failed to reset FFmpeg decoder after keyframe decode error: {exc}",
                    )
                    return []
            else:
                self._warn_decode_failure(topic, f"Failed to decode FFmpeg payload: {exc}")
                return []

        if not frames:
            self.stats["packets_without_frames"] += 1
            return []

        frame = frames[-1]
        ros_encoding = self._ros_encodings.get(topic, desired_encoding)
        av_format = _ROS_IMAGE_TO_AV_FORMAT.get(ros_encoding, "bgr24")
        try:
            image_array = frame.to_ndarray(format=av_format)
        except Exception as exc:
            self.stats["decode_errors"] += 1
            logger.warning(
                "Failed to convert FFmpeg frame to ndarray on %s using format %s: %s",
                topic,
                av_format,
                exc,
            )
            return []

        if image_array.ndim == 2:
            step = int(image_array.shape[1] * image_array.dtype.itemsize)
        else:
            step = int(image_array.shape[1] * image_array.shape[2] * image_array.dtype.itemsize)

        frame_stamp_ns = self._stamp_to_ns(getattr(packet_msg, "header", None))
        msg_stamp_ns = frame_stamp_ns or int(bag_timestamp_ns)

        image_msg = Image()
        image_msg.header.stamp = rclpy.time.Time(nanoseconds=msg_stamp_ns).to_msg()
        image_msg.header.frame_id = getattr(getattr(packet_msg, "header", None), "frame_id", "")
        image_msg.height = int(image_array.shape[0])
        image_msg.width = int(image_array.shape[1])
        image_msg.encoding = ros_encoding
        image_msg.is_bigendian = bool(getattr(packet_msg, "is_bigendian", False))
        image_msg.step = step
        image_msg.data = np.ascontiguousarray(image_array).tobytes()

        self.stats["frames_emitted"] += 1
        return [(dst_topic, "sensor_msgs/msg/Image", image_msg, int(bag_timestamp_ns))]

    def _reset_codec_context(
        self,
        topic: str,
        codec_name: str,
        ros_encoding: str,
    ):
        try:
            codec_ctx = self._av.CodecContext.create(codec_name, "r")
        except Exception:
            return None
        self._codec_contexts[topic] = codec_ctx
        self._codec_names[topic] = codec_name
        self._ros_encodings[topic] = ros_encoding
        self._synced_on_keyframe[topic] = False
        return codec_ctx

    def _warn_decode_failure(self, topic: str, message: str) -> None:
        count = int(self._decode_warning_counts.get(topic, 0)) + 1
        self._decode_warning_counts[topic] = count
        if count <= 3 or count in (10, 50, 100):
            logger.warning("%s on %s (count=%d)", message, topic, count)
        else:
            logger.debug("%s on %s (count=%d)", message, topic, count)

    @staticmethod
    def _stamp_to_ns(header: Optional[object]) -> int:
        if header is None:
            return 0
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return 0
        sec = int(getattr(stamp, "sec", 0) or 0)
        nanosec = int(getattr(stamp, "nanosec", 0) or 0)
        return sec * 1_000_000_000 + nanosec

    def get_stats(self) -> Dict[str, int]:
        return dict(self.stats)
