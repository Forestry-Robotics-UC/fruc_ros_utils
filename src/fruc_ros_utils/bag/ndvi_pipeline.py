"""Stateless NDVI pipeline for MAPIR bags."""

import logging
import os
from typing import Optional

import numpy as np
import rosbag
from cv_bridge import CvBridge

from fruc_ros_utils.bag.ros1_bag_ops import (
    _discover_bags,
    _iter_bags,
    _iter_messages,
    _resolve_out_bag,
)
from fruc_ros_utils.vision.mapir_ndvi import colorize_ndvi, compute_ndvi_from_bgr, resolve_channels

logger = logging.getLogger(__name__)


def mapir_ndvi(
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
    if output_encoding not in ("32FC1", "mono8"):
        raise ValueError("output_encoding must be '32FC1' or 'mono8'")
    if not colorize_max > colorize_min:
        raise ValueError("colorize_max must be > colorize_min")
    if eps <= 0.0:
        raise ValueError("eps must be > 0")

    nir_channel, visible_channel, resolved_visible_band = resolve_channels(
        filter_set=filter_set,
        nir_channel=nir_channel,
        visible_channel=visible_channel,
    )
    visible_band_name = visible_band_name or resolved_visible_band

    bag_files = _discover_bags(in_path)
    multiple = len(bag_files) > 1
    bridge = CvBridge()

    for bag_file in _iter_bags(bag_files, desc="MAPIR NDVI bags"):
        out_bag_file = _resolve_out_bag(out_path, bag_file, multiple)
        converted = 0
        failures = 0

        with rosbag.Bag(out_bag_file, "w") as out_bag, rosbag.Bag(bag_file, "r") as in_bag:
            for _, msg, t in _iter_messages(
                in_bag,
                desc=f"NDVI {os.path.basename(bag_file)}",
                topics=[image_topic],
            ):
                try:
                    bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    ndvi = compute_ndvi_from_bgr(
                        bgr,
                        nir_channel=nir_channel,
                        visible_channel=visible_channel,
                        eps=eps,
                    )
                    if output_encoding == "mono8":
                        ndvi_image = np.clip(
                            np.rint((ndvi + 1.0) * 127.5), 0.0, 255.0
                        ).astype(np.uint8, copy=False)
                    else:
                        ndvi_image = ndvi
                    out_msg = bridge.cv2_to_imgmsg(ndvi_image, encoding=output_encoding)
                    out_msg.header = msg.header
                    out_bag.write(output_topic, out_msg, t)

                    if publish_color:
                        color_bgr = colorize_ndvi(
                            ndvi,
                            colormap=colormap,
                            colorize_min=colorize_min,
                            colorize_max=colorize_max,
                            custom_colormap=custom_colormap,
                        )
                        color_msg = bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
                        color_msg.header = msg.header
                        out_bag.write(color_topic, color_msg, t)

                    converted += 1
                except Exception as exc:
                    failures += 1
                    logger.warning(
                        "Failed to derive NDVI for %s in %s at t=%.6f: %s",
                        image_topic,
                        bag_file,
                        t.to_sec() if hasattr(t, "to_sec") else float(t),
                        exc,
                    )

        if converted == 0:
            logger.warning(
                "No NDVI frames written for %s from topic %s", bag_file, image_topic
            )
        else:
            logger.info(
                "Wrote %d NDVI frames to %s "
                "(image_topic=%s output_topic=%s publish_color=%s color_topic=%s "
                "filter_set=%s visible_band=%s failures=%d)",
                converted, out_bag_file, image_topic, output_topic,
                publish_color, color_topic, filter_set, visible_band_name, failures,
            )
