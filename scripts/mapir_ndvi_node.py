#!/usr/bin/env python3
"""Compute a simple NDVI-style image from /mapir/image_raw."""

import os
from pathlib import Path
import sys
import time
from typing import Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

try:
    from tqdm import tqdm
except Exception:
    tqdm = None

SRC_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SRC_ROOT) not in sys.path and SRC_ROOT.exists():
    sys.path.insert(0, str(SRC_ROOT))

from vision.mapir_ndvi import colorize_ndvi, compute_ndvi_from_bgr, resolve_channels


class MapirNdviNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/mapir/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/mapir/indices/ndvi")
        self.output_encoding = rospy.get_param("~output_encoding", "32FC1").strip()
        self.publish_color = bool(rospy.get_param("~publish_color", True))
        self.color_topic = rospy.get_param("~color_topic", "/mapir/indices_color/ndvi")
        self.colormap = rospy.get_param("~colormap", "plant_health").strip().lower()
        self.custom_colormap = rospy.get_param("~custom_colormap", "")
        self.colorize_min = float(rospy.get_param("~colorize_min", -1.0))
        self.colorize_max = float(rospy.get_param("~colorize_max", 1.0))
        self.filter_set = rospy.get_param("~filter_set", "OCN")
        self.eps = float(rospy.get_param("~eps", 1.0e-6))
        self.queue_size = int(rospy.get_param("~queue_size", 1))
        self.report_every_n = int(rospy.get_param("~report_every_n", 120))
        self.show_progress = bool(
            rospy.get_param("~show_progress", bool(sys.stderr.isatty()))
        )
        self.preview_every_sec = float(rospy.get_param("~preview_every_sec", 60.0))
        self.preview_show = bool(
            rospy.get_param("~preview_show", bool(os.environ.get("DISPLAY")))
        )
        self.preview_save = bool(rospy.get_param("~preview_save", False))
        self.preview_dir = Path(
            str(rospy.get_param("~preview_dir", "/tmp/mapir_ndvi_preview"))
        ).expanduser()
        self.preview_window_name = str(
            rospy.get_param("~preview_window_name", "mapir_ndvi_preview")
        )

        self.nir_channel, self.visible_channel, self.visible_band_name = resolve_channels(
            filter_set=self.filter_set,
            nir_channel=int(rospy.get_param("~nir_channel", -1)),
            visible_channel=int(rospy.get_param("~visible_channel", -1)),
        )
        self.visible_band_name = rospy.get_param("~visible_band_name", self.visible_band_name)

        if self.output_encoding not in ("32FC1", "mono8"):
            raise ValueError("~output_encoding must be '32FC1' or 'mono8'")
        if self.queue_size < 1:
            raise ValueError("~queue_size must be >= 1")
        if self.report_every_n < 0:
            raise ValueError("~report_every_n must be >= 0")
        if self.preview_every_sec < 0.0:
            raise ValueError("~preview_every_sec must be >= 0")
        if not self.colorize_max > self.colorize_min:
            raise ValueError("~colorize_max must be > ~colorize_min")
        self.pub = rospy.Publisher(
            self.output_topic,
            Image,
            queue_size=self.queue_size,
        )
        self.color_pub = (
            rospy.Publisher(
                self.color_topic,
                Image,
                queue_size=self.queue_size,
            )
            if self.publish_color
            else None
        )
        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=self.queue_size,
        )

        self.frame_count = 0
        self._last_preview_wall_time = 0.0
        self.progress = None
        rospy.on_shutdown(self._close_progress)
        if self.show_progress and tqdm is not None:
            self.progress = tqdm(
                total=None,
                desc="mapir_ndvi",
                unit="frame",
                dynamic_ncols=True,
                mininterval=0.5,
            )
        rospy.loginfo(
            (
                "mapir_ndvi_node: in=%s out=%s encoding=%s color_out=%s "
                "publish_color=%s colormap=%s show_progress=%s preview_every_sec=%.1f "
                "preview_show=%s preview_save=%s filter_set=%s "
                "nir_channel=%d visible_channel=%d visible_band=%s"
            ),
            self.image_topic,
            self.output_topic,
            self.output_encoding,
            self.color_topic,
            self.publish_color,
            self.colormap,
            self.show_progress,
            self.preview_every_sec,
            self.preview_show,
            self.preview_save,
            self.filter_set,
            self.nir_channel,
            self.visible_channel,
            self.visible_band_name,
        )
        if str(self.filter_set).strip().upper() == "OCN":
            rospy.loginfo(
                "mapir_ndvi_node: OCN mode uses the orange band as the NDVI visible-band proxy."
            )

    def _image_callback(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(
                5.0,
                f"mapir_ndvi_node: failed to decode {self.image_topic} as bgr8: {exc}",
            )
            return

        if bgr.ndim != 3 or bgr.shape[2] != 3:
            rospy.logwarn_throttle(
                5.0,
                f"mapir_ndvi_node: expected 3-channel image, got shape={getattr(bgr, 'shape', None)}",
            )
            return

        ndvi = self._compute_ndvi(bgr)
        out_msg = self._to_ros_image(ndvi, msg)
        self.pub.publish(out_msg)

        needs_color_render = self.color_pub is not None or (
            self.preview_every_sec > 0.0 and (self.preview_show or self.preview_save)
        )
        preview_color = None
        if needs_color_render:
            preview_color = self._colorize_ndvi(ndvi)

        if self.color_pub is not None and preview_color is not None:
            color_msg = self.bridge.cv2_to_imgmsg(preview_color, encoding="bgr8")
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)

        self.frame_count += 1
        if self.progress is not None:
            self.progress.update(1)
        if preview_color is not None:
            self._maybe_emit_preview(bgr, preview_color, ndvi)
        if self.report_every_n and self.frame_count % self.report_every_n == 0:
            rospy.loginfo(
                (
                    "mapir_ndvi_node: published %d frames; "
                    "ndvi[min=%.3f max=%.3f mean=%.3f]"
                ),
                self.frame_count,
                float(np.nanmin(ndvi)),
                float(np.nanmax(ndvi)),
                float(np.nanmean(ndvi)),
            )

    def _compute_ndvi(self, bgr: np.ndarray) -> np.ndarray:
        return compute_ndvi_from_bgr(
            bgr,
            nir_channel=self.nir_channel,
            visible_channel=self.visible_channel,
            eps=self.eps,
        )

    def _to_ros_image(self, ndvi: np.ndarray, src_msg: Image) -> Image:
        if self.output_encoding == "mono8":
            scaled = np.rint((ndvi + 1.0) * 127.5)
            image_out = np.clip(scaled, 0.0, 255.0).astype(np.uint8, copy=False)
        else:
            image_out = ndvi

        out_msg = self.bridge.cv2_to_imgmsg(image_out, encoding=self.output_encoding)
        out_msg.header = src_msg.header
        return out_msg

    def _maybe_emit_preview(
        self,
        source_bgr: np.ndarray,
        ndvi_color_bgr: np.ndarray,
        ndvi: np.ndarray,
    ) -> None:
        if self.preview_every_sec <= 0.0:
            return
        now = time.monotonic()
        if (now - self._last_preview_wall_time) < self.preview_every_sec:
            return
        self._last_preview_wall_time = now

        preview = self._compose_preview(source_bgr, ndvi_color_bgr, ndvi)
        if self.preview_show:
            try:
                cv2.imshow(self.preview_window_name, preview)
                cv2.waitKey(1)
            except cv2.error as exc:
                rospy.logwarn_throttle(
                    30.0,
                    f"mapir_ndvi_node: preview window disabled due to OpenCV/X11 error: {exc}",
                )
                self.preview_show = False

        if self.preview_save:
            self.preview_dir.mkdir(parents=True, exist_ok=True)
            out_path = self.preview_dir / f"preview_{self.frame_count:06d}.png"
            cv2.imwrite(str(out_path), preview)
            rospy.loginfo("mapir_ndvi_node: wrote preview %s", out_path)

    def _compose_preview(
        self,
        source_bgr: np.ndarray,
        ndvi_color_bgr: np.ndarray,
        ndvi: np.ndarray,
    ) -> np.ndarray:
        left = source_bgr.copy()
        right = ndvi_color_bgr.copy()
        cv2.putText(
            left,
            "MAPIR source",
            (16, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            right,
            "NDVI preview",
            (16, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        stats = (
            f"min={float(np.nanmin(ndvi)):.3f} "
            f"max={float(np.nanmax(ndvi)):.3f} "
            f"mean={float(np.nanmean(ndvi)):.3f}"
        )
        cv2.putText(
            right,
            stats,
            (16, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return np.hstack((left, right))

    def _colorize_ndvi(self, ndvi: np.ndarray) -> np.ndarray:
        return colorize_ndvi(
            ndvi,
            colormap=self.colormap,
            colorize_min=self.colorize_min,
            colorize_max=self.colorize_max,
            custom_colormap=self.custom_colormap,
        )

    def _close_progress(self) -> None:
        if self.progress is not None:
            self.progress.close()
            self.progress = None
        if self.preview_show:
            try:
                cv2.destroyWindow(self.preview_window_name)
            except cv2.error:
                pass


def main() -> None:
    rospy.init_node("mapir_ndvi_node")
    MapirNdviNode()
    rospy.spin()


if __name__ == "__main__":
    main()
