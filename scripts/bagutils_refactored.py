#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Bag Utilities (Refactored with illumination + optional egomotion & bag saving)

Author: Duda Andrada
Maintainer: Duda Andrada
"""

from __future__ import annotations
import xml.etree.ElementTree as ET
import argparse
import logging
import os
import pathlib
import random
import sys
import yaml
from dataclasses import dataclass
from typing import Dict, List, Optional
from collections import deque

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, Image, CameraInfo  # imported for types only; do NOT use isinstance with ROS1
from tqdm import tqdm

from navsat_tools import NavSatExporter

# Illumination module
from illumination import IlluminationEnhancer, IlluminationConfig

try:
    import argcomplete
except Exception:  # pragma: no cover
    argcomplete = None


# --------------------------- Helpers ----------------------------------------

def make_logger(name: str = "rosutils", level: int = logging.INFO) -> logging.Logger:
    class _ColorFormatter(logging.Formatter):
        _COL = {
            "INFO": "\033[90m",          # gray
            "WARNING": "\033[33m",       # yellow
            "ERROR": "\033[31m",         # red
            "DEBUG": "\033[38;5;208m",   # orange
        }
        _RESET = "\033[0m"

        def format(self, record: logging.LogRecord) -> str:
            base = "%(asctime)s - %(levelname)s - %(message)s"
            msg = logging.Formatter(base).format(record)
            col = self._COL.get(record.levelname, "")
            return f"{col}{msg}{self._RESET}" if col else msg

    logger = logging.getLogger(name)
    if not logger.handlers:
        logger.setLevel(level)
        handler = logging.StreamHandler(stream=sys.stdout)
        handler.setFormatter(_ColorFormatter())
        logger.addHandler(handler)
    return logger

def build_R_cam_imu_per_topic(
    sample_bag_path: str,
    topics: List[str],
    imu_frame: str,
    urdf_path: Optional[str] = None,
    tf_static_topic: str = "/tf_static",
) -> Dict[str, np.ndarray]:
    """
    For each image topic:
      - read its camera frame_id from the first Image or CameraInfo msg
      - compute R_cam_imu from either URDF or /tf_static (URDF preferred if given)
    """
    # self.logger.debug(f"Building R_cam_imu for topics={topics},"
        # f"imu_frame={imu_frame}, urdf={urdf_path}, tf_topic={tf_static_topic}")

    def _rot_from_T(T4):
        R = T4[:3, :3]
        # ensure proper rotation (tiny numerical fix)
        U, _, Vt = np.linalg.svd(R)
        return U @ Vt

    # 1) discover frame_ids per topic
    topic_frame = {}
    with rosbag.Bag(sample_bag_path, "r") as b:
        need = set(topics)
        for topic, msg, _ in b.read_messages(topics=topics + [tf_static_topic]):
            if topic in topics and topic not in topic_frame:
                mtype = getattr(msg, "_type", "")
                if "Image" in mtype and hasattr(msg, "header"):
                    topic_frame[topic] = msg.header.frame_id
                    # self.logger.debug(f"Discovered topic->frame mapping: {topic_frame}")
                elif "CameraInfo" in mtype and hasattr(msg, "header"):
                    topic_frame[topic] = msg.header.frame_id
            if len(topic_frame) == len(set(topics)):
                break

    # 2) get transforms source
    tf_graph = {}
    if urdf_path:
        # we will query URDF on demand per topic
        pass
    else:
        # build graph from /tf_static in the same bag
        import tf.transformations as tft  # ROS tf1 helpers if available; fallback below
        with rosbag.Bag(sample_bag_path, "r") as b:
            for _, msg, _ in b.read_messages(topics=[tf_static_topic]):
                for ts in msg.transforms:
                    p = ts.header.frame_id
                    c = ts.child_frame_id
                    t = np.array([ts.transform.translation.x,
                                  ts.transform.translation.y,
                                  ts.transform.translation.z], dtype=float)
                    q = np.array([ts.transform.rotation.x,
                                  ts.transform.rotation.y,
                                  ts.transform.rotation.z,
                                  ts.transform.rotation.w], dtype=float)
                    # 4x4
                    T = np.eye(4)
                    # robust, no dependency on tf if missing:
                    q = q / (np.linalg.norm(q) + 1e-12)
                    x,y,z,w = q
                    R = np.array([
                        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
                        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
                    ], dtype=float)
                    T[:3, :3] = R
                    T[:3, 3] = t
                    tf_graph.setdefault(p, []).append((c, T))
                    tf_graph.setdefault(c, []).append((p, np.linalg.inv(T)))
                    # self.logger.debug(f"Built tf_graph with {len(tf_graph)} nodes")


    # 3) compute R(cam, imu)
    R_by_topic = {}
    for topic in topics:
        cam_frame = topic_frame.get(topic)
        if not cam_frame:
            continue
        if urdf_path:
            T = load_extrinsics_from_urdf(urdf_path, cam_frame, imu_frame)
            R_by_topic[topic] = _rot_from_T(T)
        else:
            # BFS over tf_graph
            from collections import deque
            q = deque([(cam_frame, np.eye(4))])
            vis = {cam_frame}
            found = None
            while q:
                node, Tacc = q.popleft()
                if node == imu_frame:
                    found = Tacc
                    break
                for nb, Te in tf_graph.get(node, []):
                    if nb not in vis:
                        vis.add(nb)
                        q.append((nb, Tacc @ Te))
            if found is None:
                raise ValueError(f"No static TF path {cam_frame} -> {imu_frame}")
            R_by_topic[topic] = _rot_from_T(found)
            # self.logger.debug(f"Computed R_cam_imu for {topic}:\n{R_by_topic[topic]}")

    return R_by_topic

def load_extrinsics_yaml(path: str) -> np.ndarray:
    """Load R_cam_imu (3x3) from a YAML file with key 'rotation_matrix'."""
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    R = data.get("rotation_matrix")
    if R is None:
        raise ValueError("extrinsics YAML must contain 'rotation_matrix'")
    R = np.array(R, dtype=float)
    if R.size == 9:
        R = R.reshape(3, 3)
    if R.shape != (3, 3):
        raise ValueError("rotation_matrix must be 3x3")
    return R

def parse_origin(element):
    """Parse <origin xyz="" rpy=""> into 4x4 transform matrix."""
    xyz = element.attrib.get("xyz", "0 0 0")
    rpy = element.attrib.get("rpy", "0 0 0")
    # self.logger.debug(f"Parsing origin: xyz={element.attrib.get('xyz','0 0 0')},"
        # f"rpy={element.attrib.get('rpy','0 0 0')}")

    tx, ty, tz = map(float, xyz.split())
    rr, pp, yy = map(float, rpy.split())

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rr), -np.sin(rr)],
                   [0, np.sin(rr), np.cos(rr)]])
    Ry = np.array([[np.cos(pp), 0, np.sin(pp)],
                   [0, 1, 0],
                   [-np.sin(pp), 0, np.cos(pp)]])
    Rz = np.array([[np.cos(yy), -np.sin(yy), 0],
                   [np.sin(yy), np.cos(yy), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    # logger.debug(f"Generated transform:\n{T}")

    return T


def load_extrinsics_from_urdf(urdf_path: str, link_a: str, link_b: str) -> np.ndarray:
    """
    Load extrinsics (4x4 homogeneous transform) between two links from a URDF.
    Traverses the joint tree and composes transforms.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    # self.logger.debug(f"Parsing URDF {urdf_path}, searching path {link_a} -> {link_b}")

    adjacency = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin_elem = joint.find("origin")
        T = parse_origin(origin_elem) if origin_elem is not None else np.eye(4)
        adjacency.setdefault(parent, []).append((child, T))
        adjacency.setdefault(child, []).append((parent, np.linalg.inv(T)))
        # self.logger.debug(f"Joint {joint.attrib.get('name','?')} connects {parent} -> {child}")

    queue = deque([(link_a, np.eye(4))])
    visited = {link_a}

    while queue:
        node, T_acc = queue.popleft()
        if node == link_b:
            # self.logger.debug(f"Found path {link_a} -> {link_b}, transform=\n{T_acc}")
            return T_acc
        for neighbor, T_edge in adjacency.get(node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, T_acc @ T_edge))

    raise ValueError(f"No path found between {link_a} and {link_b} in URDF {urdf_path}")

@dataclass
class BlurConfig:
    min_threshold: int = 40
    start_threshold: int = 100
    target_candidates_factor: int = 2


# --------------------------- Core class -------------------------------------

class RosbagUtils:
    def __init__(self, log_level: int = logging.INFO) -> None:
        self.logger = make_logger(level=log_level)
        self.bridge = CvBridge()

    # -------- durations --------
    def calculate_bag_duration(self, in_path: str, *_) -> float:
        self.logger.debug(f"Opening bag {in_path} for duration calculation")

        with rosbag.Bag(in_path, "r") as bag:
            duration = bag.get_end_time() - bag.get_start_time()
            self.logger.debug(f"Bag {in_path} start={bag.get_start_time()}, \
            end={bag.get_end_time()}, duration={duration}")
        self.logger.info("Duration of %s: %.3f s", os.path.basename(in_path), duration)
        return duration

    def sum_bag_durations(self, in_path: str, *_) -> float:
        total = 0.0
        bag_files = [f for f in os.listdir(in_path) if f.endswith(".bag")]
        self.logger.debug(f"Summing durations for {len(bag_files)} bags in {in_path}")

        for bag_file in tqdm(bag_files, desc="Summing durations"):
            total += self.calculate_bag_duration(os.path.join(in_path, bag_file))
        self.logger.info("Total duration of %d bags: %.3f s", len(bag_files), total)
        return total

    # -------- topic ops --------
    def remove_topic(self, in_path: str, out_path: str, topics: List[str]) -> None:
        self.logger.debug(f"Removing topics {topics} from {in_path}, writing to {out_path}")
        with rosbag.Bag(out_path, "w") as out_bag, rosbag.Bag(in_path, "r") as in_bag:
            total = in_bag.get_message_count()
            for topic, msg, t in tqdm(in_bag.read_messages(),
                                      total=total,
                                      desc=f"Removing topics from {os.path.basename(in_path)}"):
                if topic not in topics:
                    self.logger.debug(f"Writing topic {topic}, skipped={topic in topics}")
                    out_bag.write(topic, msg, t)
        self.logger.info("Wrote %s", out_path)

    def change_frame_id(self, in_path: str, out_path: str, topics: List[str], new_frame_id: str) -> None:
        topic = topics[0]
        with rosbag.Bag(out_path, "w") as out_bag, rosbag.Bag(in_path, "r") as in_bag:
            total = in_bag.get_message_count(topic_filters=[topic])
            for _, msg, t in tqdm(in_bag.read_messages(topics=[topic]),
                                  total=total,
                                  desc=f"Changing frame_id on {topic}"):
                # Do NOT use isinstance with ROS1; rely on header presence
                if hasattr(msg, "header") and hasattr(msg.header, "frame_id"):
                    self.logger.debug(f"Changing frame_id on topic={topic} to {new_frame_id}")
                    msg.header.frame_id = new_frame_id
                out_bag.write(topic, msg, t)
        self.logger.info("Wrote %s", out_path)

    def print_topic_sizes(self, in_path: str, *_) -> Dict[str, int]:
        sizes: Dict[str, int] = {}
        with rosbag.Bag(in_path, "r") as bag:
            total = bag.get_message_count()
            self.logger.debug(f"Bag {in_path} total messages={total}")
            for topic, msg, _ in tqdm(bag.read_messages(raw=True),
                                      total=total,
                                      desc=f"Topic sizes in {os.path.basename(in_path)}"):
                # msg is (connection_header, serialized_bytes)
                sizes[topic] = sizes.get(topic, 0) + len(msg[1])
                self.logger.debug(f"Topic {topic} accumulated size={sizes[topic]} bytes")
        for topic, size in sorted(sizes.items(), key=lambda x: x[1]):
            self.logger.info("%s: %.2f MB (%.4f GB)", topic, size / 1e6, size / 1e9)
        return sizes

    # -------- IMU NED->ENU --------
    @staticmethod
    def ned_to_enu(imu: Imu) -> Imu: 
        self.logger.debug(f"Converting IMU {imu.header.stamp} from NED->ENU")
        enu = Imu()
        enu.header = imu.header
        enu.angular_velocity.x = imu.angular_velocity.y
        enu.angular_velocity.y = imu.angular_velocity.x
        enu.angular_velocity.z = -imu.angular_velocity.z
        enu.linear_acceleration.x = imu.linear_acceleration.y
        enu.linear_acceleration.y = imu.linear_acceleration.x
        enu.linear_acceleration.z = -imu.linear_acceleration.z
        enu.orientation.x = imu.orientation.y
        enu.orientation.y = imu.orientation.x
        enu.orientation.z = -imu.orientation.z
        enu.orientation.w = imu.orientation.w
        return enu

    def convert_imu_to_enu(self, in_path: str, out_path: str, topics: List[str]) -> None:
        imu_topic = topics[0]
        self.logger.debug(f"Converting IMU topic={imu_topic}, input={in_path}, output={out_path}")
        with rosbag.Bag(out_path, "w") as out_bag, rosbag.Bag(in_path, "r") as in_bag:
            total = in_bag.get_message_count()
            for topic, msg, t in tqdm(in_bag.read_messages(),
                                      total=total,
                                      desc=f"Converting IMU in {os.path.basename(in_path)}"):
                if topic == imu_topic and hasattr(msg, "_type") and "Imu" in msg._type:
                    out_bag.write(topic, self.ned_to_enu(msg), t)
                else:
                    out_bag.write(topic, msg, t)
        self.logger.info("Wrote %s", out_path)

    # -------- Illumination --------
    def auto_illumination_from_bag(
        self,
        in_path: str,
        out_path: str,
        topics: List[str],
        report: Optional[str],
        force: bool,
        suffix: str,
        white_balance: bool,
        save_bag: Optional[str] = None,
        extrinsics_yaml: Optional[str] = None,
        exposure_time: float = 0.01,
        ego_thresh_rad: float = 0.01,
        imu_frame: Optional[str] = None,
        urdf_path: Optional[str] = None,
        tf_static_topic: str = "/tf_static",
        preserve_bayer: bool = True,
    ) -> None:
        """
        Process one or more bag files with auto-illumination.
        - Corrects images from the provided topics.
        - If 'report' is a directory, saves per-image reports + CSV there.
        - If 'save_bag' is provided, writes corrected bag(s).
        """

        self.logger.debug(f"Starting auto_illumination: in={in_path}, out={out_path}, topics={topics}")
        os.makedirs(out_path, exist_ok=True)
        if report:
            if report is True:
                report = out_path
            os.makedirs(report, exist_ok=True)

        # Illumination enhancer
        cfg = IlluminationConfig(white_balance=white_balance, ego_theta_thresh_rad=ego_thresh_rad)
        enh = IlluminationEnhancer(config=cfg, logger=self.logger)

        # Bag file discovery
        pin = pathlib.Path(in_path)
        if pin.is_dir():
            bagfiles = sorted(str(p) for p in pin.glob("*.bag*") if p.is_file())
        else:
            bagfiles = [str(pin)]
        if not bagfiles:
            self.logger.error("No bag files found in %s", in_path)
            return
        self.logger.debug(f"Bagfiles: {bagfiles}")

        # Save_bag handling
        out_bag_dir: Optional[pathlib.Path] = None
        single_out_path: Optional[str] = None
        if save_bag:
            sbp = pathlib.Path(save_bag)
            if len(bagfiles) > 1:
                if sbp.suffix == ".bag":
                    self.logger.warning(
                        "save_bag='%s' looks like a file but multiple bags found; using its parent dir.", save_bag
                    )
                    out_bag_dir = sbp.parent
                else:
                    out_bag_dir = sbp
                out_bag_dir.mkdir(parents=True, exist_ok=True)
            else:
                if sbp.is_dir() or sbp.suffix != ".bag":
                    out_bag_dir = sbp
                    out_bag_dir.mkdir(parents=True, exist_ok=True)
                else:
                    single_out_path = str(sbp)

        # Build R_cam_imu maps (extrinsics)
        if extrinsics_yaml:
            R_cam_imu_map = {"default": load_extrinsics_yaml(extrinsics_yaml)}
        elif imu_frame:
            sample_bag = bagfiles[0]
            R_cam_imu_map = build_R_cam_imu_per_topic(
                sample_bag_path=sample_bag,
                topics=topics,
                imu_frame=imu_frame,
                urdf_path=urdf_path,
                tf_static_topic=tf_static_topic,
            )
        else:
            R_cam_imu_map = {}


        # Process each bag
        with tqdm(total=len(bagfiles), desc="Bags", unit="bag", position=0) as bag_pbar:
            
            total_corrected = 0
            for bag_path in bagfiles:
                bag_pbar.update(1)

                # Resolve output path
                if save_bag:
                    if single_out_path:
                        out_bag_path = single_out_path
                    else:
                        base = os.path.basename(bag_path)
                        stem = os.path.splitext(base)[0]
                        out_bag_path = str((out_bag_dir / f"_corrected.bag"))
                else:
                    out_bag_path = None

                # Call into IlluminationEnhancer
                summary = enh.process_bag(
                    bag_path=bag_path,
                    out_bag_path=out_bag_path,
                    topics=topics,
                    report_dir=report,
                    force=force,
                    exposure_time=exposure_time,
                    R_cam_imu_map=R_cam_imu_map,
                    preserve_bayer=preserve_bayer,
                )
                corrected = summary.get("corrected_images", 0)
                total_corrected += corrected
                self.logger.info("Finished illumination for %s", bag_path)
                self.logger.info(
                    "Summary — corrected: %d / %d images, egomotion corrections: %d",
                    corrected,
                    summary.get("total_images", 0),
                    summary.get("egomotion_corrections", 0),
                )

                # Update outer progress bar
                bag_pbar.update(1)
                bag_pbar.set_postfix({"corrected_total": total_corrected})


# --------------------------- CLI Parser -------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS bag utilities (refactored)")

    # ---- Common argument groups (ANYTHING used more than once goes here) ----
    common_in = argparse.ArgumentParser(add_help=False)
    common_in.add_argument("--in", dest="in_path", required=True,
                           help="Input .bag file (for file ops) or folder (for folder ops)")

    common_out = argparse.ArgumentParser(add_help=False)
    common_out.add_argument("--out", dest="out_path", required=True,
                            help="Output file or folder")

    common_topics = argparse.ArgumentParser(add_help=False)
    common_topics.add_argument("--topics", nargs="+", required=True,
                               help="ROS topics to process (images will be corrected if in this list)")

    common_extrinsics = argparse.ArgumentParser(add_help=False)
    common_extrinsics.add_argument("--extrinsics-yaml", dest="extrinsics_yaml",
                                   help="YAML with 3x3 'rotation_matrix' (R_cam_imu)")

    # Shared illumination args (appear in both illumination commands)
    common_illum = argparse.ArgumentParser(add_help=False)
    common_illum.add_argument("--report", help="Directory for per-image illumination reports")
    common_illum.add_argument("--force", action="store_true", help="Always apply correction")
    common_illum.add_argument("--suffix", default="", help="Suffix for outputs")
    common_illum.add_argument("--no-wb", action="store_true", help="Disable white balance")
    common_illum.add_argument("--exposure-time", type=float, default=0.01,
                              help="Exposure time in seconds")
    common_illum.add_argument("--ego-thresh-rad", type=float, default=0.01,
                              help="Rotation threshold to apply egomotion (rad)")
    common_illum.add_argument("--imu-frame", help="IMU frame for auto extrinsics (used with URDF or /tf_static)")
    common_illum.add_argument("--urdf", help="URDF path (if not set, use /tf_static)")
    common_illum.add_argument("--tf-static-topic", default="/tf_static", help="TF static topic to read transforms from")
    common_illum.add_argument("--preserve-bayer", action="store_true", help="Keep Bayer encoding in bags instead of demosaicing")
    sub = parser.add_subparsers(dest="cmd", required=True)

    # -------- auto_illumination --------
    sp = sub.add_parser(
        "auto_illumination",
        parents=[common_in, common_out, common_topics, common_extrinsics, common_illum],
        help="Extract images and auto-correct illumination; optional bag save"
    )
    sp.add_argument("--save-bag", dest="save_bag",
                    help="If set, write a new bag with corrected images")

    # -------- preview_illumination --------
    sp = sub.add_parser(
        "preview_illumination",
        parents=[common_in, common_out, common_topics, common_extrinsics, common_illum],
        help="Preview illumination correction on random samples from a folder of bags"
    )
    sp.add_argument("--num-messages", type=int, default=50, help="Messages per bag to preview")
    sp.add_argument("--num-bags", type=int, default=3, help="Number of bags to sample")
    sp.add_argument("--seed", type=int, default=42, help="Random seed")
    sp.set_defaults(suffix="_preview")  # sensible default for preview

    # -------- calculate_bag_duration --------
    sp = sub.add_parser(
        "calculate_bag_duration",
        parents=[common_in],
        help="Calculate duration of a single bag"
    )

    # -------- sum_bag_durations --------
    sp = sub.add_parser(
        "sum_bag_durations",
        parents=[common_in],
        help="Sum durations of all .bag files in a folder"
    )

    # -------- remove_topic --------
    sp = sub.add_parser(
        "remove_topic",
        parents=[common_in, common_out, common_topics],
        help="Copy all topics except those listed in --topics"
    )
    # -------- urdf_extrinsics --------
    sp = sub.add_parser(
            "urdf_extrinsics",
            help="Extract extrinsics (transform) between two links from a URDF"
        )
    sp.add_argument("--urdf", required=True, help="Path to URDF file")
    sp.add_argument("--from", dest="link_a", required=True, help="Source link")
    sp.add_argument("--to", dest="link_b", required=True, help="Target link")
    fmt_group = sp.add_mutually_exclusive_group()
    fmt_group.add_argument("--rotation-only", action="store_true",
                           help="Output only the 3x3 rotation matrix")
    fmt_group.add_argument("--translation-only", action="store_true",
                           help="Output only the 3x1 translation vector")

    # -------- change_frame_id --------
    sp = sub.add_parser(
        "change_frame_id",
        parents=[common_in, common_out, common_topics],
        help="Change frame_id on the first topic in --topics"
    )
    sp.add_argument("--new-frame-id", required=True, help="New frame_id to set")

    # -------- print_topic_sizes --------
    sp = sub.add_parser(
        "print_topic_sizes",
        parents=[common_in],
        help="Print cumulative serialized sizes per topic in a bag"
    )

    # -------- convert_imu_to_enu --------
    sp = sub.add_parser(
        "convert_imu_to_enu",
        parents=[common_in, common_out, common_topics],
        help="Convert IMU on the first topic in --topics from NED to ENU"
    )

    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        help="Logging verbosity")

    if argcomplete:
        argcomplete.autocomplete(parser)
    return parser


# --------------------------- Dispatcher -------------------------------------

def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    level = getattr(logging, args.log_level)
    bu = RosbagUtils(log_level=level)

    if args.cmd == "auto_illumination":
        bu.auto_illumination_from_bag(
            args.in_path,
            args.out_path,
            args.topics,
            report=args.report,
            force=args.force,
            suffix=args.suffix,
            white_balance=not args.no_wb,
            save_bag=args.save_bag,
            extrinsics_yaml=args.extrinsics_yaml,
            exposure_time=args.exposure_time,
            ego_thresh_rad=args.ego_thresh_rad,
            imu_frame=args.imu_frame,
            urdf_path=args.urdf,
            tf_static_topic=args.tf_static_topic,
            preserve_bayer=args.preserve_bayer,
        )
    # elif args.cmd == "preview_illumination":
    #     bu.preview_illumination(
    #         args.in_path, args.out_path, args.topics,
    #         report=args.report, force=args.force,
    #         suffix=args.suffix, white_balance=not args.no_wb,
    #         num_messages=args.num_messages,
    #         num_bags=args.num_bags, seed=args.seed,
    #         extrinsics_yaml=args.extrinsics_yaml,
    #         exposure_time=args.exposure_time,
    #         ego_thresh_rad=args.ego_thresh_rad,
    #         imu_frame=args.imu_frame,
    #         urdf_path=args.urdf,
    #         tf_static_topic=args.tf_static_topic,
    #         preserve_bayer=args.preserve_bayer,
    #     )

    elif args.cmd == "calculate_bag_duration":
        bu.calculate_bag_duration(args.in_path)

    elif args.cmd == "sum_bag_durations":
        bu.sum_bag_durations(args.in_path)

    elif args.cmd == "remove_topic":
        bu.remove_topic(args.in_path, args.out_path, args.topics)

    elif args.cmd == "change_frame_id":
        bu.change_frame_id(args.in_path, args.out_path, args.topics, args.new_frame_id)

    elif args.cmd == "print_topic_sizes":
        bu.print_topic_sizes(args.in_path)

    elif args.cmd == "convert_imu_to_enu":
        bu.convert_imu_to_enu(args.in_path, args.out_path, args.topics)
    elif args.cmd == "urdf_extrinsics":
        T = load_extrinsics_from_urdf(args.urdf, args.link_a, args.link_b)
        np.set_printoptions(precision=4, suppress=True)
        if args.rotation_only:
            print(f"Rotation {args.link_a} -> {args.link_b}:\n{T[:3, :3]}")
        elif args.translation_only:
            print(f"Translation {args.link_a} -> {args.link_b}:\n{T[:3, 3]}")
        else:
            print(f"Transform {args.link_a} -> {args.link_b}:\n{T}")

    else:
        parser.error(f"Unknown command: {args.cmd}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
