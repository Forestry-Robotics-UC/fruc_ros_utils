#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Extract sharp images from ROS bag files. Uses sharpness metrics to filter
#   out blurred frames and saves a random balanced set across bags.

import os
import pathlib
import random
import math
import logging
from typing import List

import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm

from utils.logging_utils import get_logger
from vision.image_utils import demosaic_bayer, is_bayer
from utils.metrics import sharpness_score, auto_threshold

logger = get_logger(__name__, level="INFO")


# --------------------------- Core Function ----------------------------------

def extract_sharp_images(
    bagfiles: List[str],
    topic: str,
    out_dir: str,
    max_images: int,
    method: str = "tenengrad",
    threshold: float = None,
    auto_percentile: float = 80.0,
    num_bags: int = None,
    seed: int = 42,
) -> None:
    """
    Extract sharp images across multiple bags with fair distribution.

    Args:
        bagfiles: list of bag paths
        topic: ROS image topic
        out_dir: directory to save extracted images
        max_images: global cap on number of images
        method: sharpness metric
        threshold: optional fixed threshold
        auto_percentile: percentile cutoff if threshold=auto
        num_bags: number of bags to randomly sample (default all)
        seed: random seed
    """
    random.seed(seed)
    out_path = pathlib.Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    if num_bags and num_bags < len(bagfiles):
        bagfiles = random.sample(bagfiles, num_bags)

    # quota distribution across bags
    per_bag_quota = math.ceil(max_images / len(bagfiles))

    bridge = CvBridge()

    total_saved = 0
    leftovers = 0

    for i, bag_path in enumerate(bagfiles, 1):
        bag = rosbag.Bag(bag_path, "r")
        images = []

        for _, msg, t in bag.read_messages(topics=[topic]):
            try:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if is_bayer(msg.encoding):
                    cv_img = demosaic_bayer(cv_img, msg.encoding)
                score = sharpness_score(cv_img, method=method)
                images.append((t.to_sec(), cv_img, score))
            except Exception as e:
                logger.warning("Failed to convert image in %s: %s", bag_path, e)

        bag.close()

        if not images:
            logger.warning("No images found in %s", bag_path)
            continue

        scores = [s for _, _, s in images]
        if threshold is None:
            cutoff = auto_threshold(scores, auto_percentile)
        else:
            cutoff = threshold

        sharp = [(t, img) for (t, img, s) in images if s >= cutoff]

        # pick up to per_bag_quota
        if len(sharp) > per_bag_quota:
            sharp = random.sample(sharp, per_bag_quota)

        # save
        for t, img in sharp:
            fname = out_path / f"bag{i}_{t:.3f}.png"
            cv2.imwrite(str(fname), img)
        total_saved += len(sharp)

        logger.info("Saved %d sharp images from %s", len(sharp), bag_path)

        # track leftover if under quota
        if len(sharp) < per_bag_quota:
            leftovers += per_bag_quota - len(sharp)

    # redistribute leftovers to remaining bags if needed
    if leftovers > 0 and total_saved < max_images:
        logger.info("Redistributing %d leftover slots", leftovers)
        for bag_path in bagfiles:
            if total_saved >= max_images:
                break
            bag = rosbag.Bag(bag_path, "r")
            extra = []
            for _, msg, t in bag.read_messages(topics=[topic]):
                try:
                    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    if is_bayer(msg.encoding):
                        cv_img = demosaic_bayer(cv_img, msg.encoding)
                    score = sharpness_score(cv_img, method=method)
                    if score >= cutoff:
                        extra.append((t.to_sec(), cv_img))
                except Exception:
                    continue
            bag.close()
            random.shuffle(extra)
            take = min(leftovers, len(extra))
            for t, img in extra[:take]:
                fname = out_path / f"extra_{t:.3f}.png"
                cv2.imwrite(str(fname), img)
            total_saved += take
            leftovers -= take

    logger.info("Done. Total sharp images saved: %d", total_saved)


# --------------------------- CLI --------------------------------------------

def build_parser():
    import argparse
    try:
        import argcomplete
    except ImportError:
        argcomplete = None

    parser = argparse.ArgumentParser(description="Extract sharp images from ROS bag files")
    parser.add_argument("--in", dest="in_path", required=True, help="Input bag file or folder")
    parser.add_argument("--out", dest="out_dir", required=True, help="Output directory")
    parser.add_argument("--topic", required=True, help="ROS image topic to extract")
    parser.add_argument("--max-images", type=int, default=200, help="Maximum total images to save")
    parser.add_argument("--num-bags", type=int, help="Number of bags to randomly sample (default all)")
    parser.add_argument("--threshold", type=float, default=None, help="Sharpness threshold (default: auto)")
    parser.add_argument("--auto-percentile", type=float, default=80.0, help="Percentile cutoff if threshold=auto")
    parser.add_argument("--method", default="tenengrad", choices=["tenengrad", "laplacian", "fft"], help="Sharpness metric")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])

    if argcomplete:
        argcomplete.autocomplete(parser)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    level = getattr(logging, args.log_level)
    logger.setLevel(level)

    p = pathlib.Path(args.in_path)
    if p.is_dir():
        bagfiles = sorted(str(f) for f in p.glob("*.bag*") if f.is_file())
    else:
        bagfiles = [str(p)]

    extract_sharp_images(
        bagfiles=bagfiles,
        topic=args.topic,
        out_dir=args.out_dir,
        max_images=args.max_images,
        method=args.method,
        threshold=args.threshold,
        auto_percentile=args.auto_percentile,
        num_bags=args.num_bags,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
