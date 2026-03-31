#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Utilities for handling TF extrinsics, URDF transforms, and IMU-camera calibration.
#   Provides helpers to build R_cam_imu mappings, parse extrinsics from YAML/URDF,
#   and compose transform chains from /tf_static or URDF joint trees.

import os
import yaml
import numpy as np
import rosbag
import xml.etree.ElementTree as ET

from typing import Dict, List, Optional
from collections import deque

# --------------------------------------------------------------------------- #
#                       CAMERA–IMU EXTRINSICS                                 #
# --------------------------------------------------------------------------- #

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

    def _rot_from_T(T4: np.ndarray) -> np.ndarray:
        """Ensure rotation matrix is orthogonal by SVD correction."""
        R = T4[:3, :3]
        U, _, Vt = np.linalg.svd(R)
        return U @ Vt

    def _norm(frame: str) -> str:
        """Normalize ROS frame ids (strip leading slash)."""
        return frame.lstrip("/") if frame else ""

    # ---------------- 1) Discover frame_ids per topic ----------------
    topic_frame: Dict[str, str] = {}
    needed = [t for t in topics if "image" in t.lower() or "camera" in t.lower()]
    with rosbag.Bag(sample_bag_path, "r") as b:
        for topic, msg, _ in b.read_messages(topics=needed + [tf_static_topic]):
            if topic in needed and topic not in topic_frame:
                mtype = getattr(msg, "_type", "")
                if "Image" in mtype or "CameraInfo" in mtype:
                    topic_frame[topic] = _norm(getattr(msg.header, "frame_id", ""))
            if len(topic_frame) == len(needed):
                break

    if not topic_frame:
        raise ValueError(f"No camera frames discovered in {sample_bag_path} for topics {needed}")

    # ---------------- 2) Build transform graph ----------------
    tf_graph: Dict[str, List] = {}
    if not urdf_path:
        with rosbag.Bag(sample_bag_path, "r") as b:
            for _, msg, _ in b.read_messages(topics=[tf_static_topic]):
                for ts in msg.transforms:
                    p = _norm(ts.header.frame_id)
                    c = _norm(ts.child_frame_id)
                    t = np.array([ts.transform.translation.x,
                                  ts.transform.translation.y,
                                  ts.transform.translation.z], dtype=float)
                    q = np.array([ts.transform.rotation.x,
                                  ts.transform.rotation.y,
                                  ts.transform.rotation.z,
                                  ts.transform.rotation.w], dtype=float)
                    q = q / (np.linalg.norm(q) + 1e-12)  # normalize quaternion
                    x, y, z, w = q
                    R = np.array([
                        [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
                        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
                        [2*(x*z - y*w), 2*(y*z + x*w),   1-2*(x*x+y*y)]
                    ], dtype=float)
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = t
                    tf_graph.setdefault(p, []).append((c, T))
                    tf_graph.setdefault(c, []).append((p, np.linalg.inv(T)))

    # ---------------- 3) Compute R(cam, imu) per topic ----------------
    imu_frame_n = _norm(imu_frame)
    R_by_topic: Dict[str, np.ndarray] = {}

    for topic in needed:
        cam_frame = topic_frame.get(topic)
        if not cam_frame:
            raise ValueError(f"No frame_id discovered for topic {topic} in {sample_bag_path}")

        if urdf_path:
            try:
                T = load_extrinsics_from_urdf(urdf_path, cam_frame, imu_frame_n)
                R_by_topic[topic] = _rot_from_T(T)
            except Exception as e:
                raise ValueError(f"URDF extrinsics failed for {cam_frame}->{imu_frame_n}: {e}")
        else:
            q = deque([(cam_frame, np.eye(4))])
            vis = {cam_frame}
            found = None
            while q:
                node, Tacc = q.popleft()
                if node == imu_frame_n:
                    found = Tacc
                    break
                for nb, Te in tf_graph.get(node, []):
                    if nb not in vis:
                        vis.add(nb)
                        q.append((nb, Tacc @ Te))
            if found is None:
                raise ValueError(f"No static TF path {cam_frame} -> {imu_frame_n}")
            R_by_topic[topic] = _rot_from_T(found)

    return R_by_topic


# --------------------------------------------------------------------------- #
#                           YAML EXTRINSICS                                   #
# --------------------------------------------------------------------------- #

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

# --------------------------------------------------------------------------- #
#                         URDF EXTRINSICS                                     #
# --------------------------------------------------------------------------- #

def parse_origin(element) -> np.ndarray:
    """Parse <origin xyz="" rpy=""> into 4x4 transform matrix."""
    xyz = element.attrib.get("xyz", "0 0 0")
    rpy = element.attrib.get("rpy", "0 0 0")

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

    return T


def load_extrinsics_from_urdf(urdf_path: str, link_a: str, link_b: str) -> np.ndarray:
    """
    Load extrinsics (4x4 homogeneous transform) between two links from a URDF.
    Traverses the joint tree and composes transforms.
    Raises descriptive ValueError if links are missing or no path is found.
    """
    import xml.etree.ElementTree as ET
    import numpy as np
    from collections import deque

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Collect all link names
    all_links = [elem.attrib["name"] for elem in root.findall("link")]

    missing = []
    if link_a not in all_links:
        missing.append(f"link_a '{link_a}'")
    if link_b not in all_links:
        missing.append(f"link_b '{link_b}'")

    if missing:
        raise ValueError(
            f"Missing {', '.join(missing)} in URDF {urdf_path}. "
            f"Available links: {', '.join(all_links)}"
        )

    # Build adjacency graph
    adjacency: Dict[str, List] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin_elem = joint.find("origin")
        T = parse_origin(origin_elem) if origin_elem is not None else np.eye(4)
        adjacency.setdefault(parent, []).append((child, T))
        adjacency.setdefault(child, []).append((parent, np.linalg.inv(T)))

    # BFS search for a path
    queue = deque([(link_a, np.eye(4))])
    visited = {link_a}

    while queue:
        node, T_acc = queue.popleft()
        if node == link_b:
            return T_acc
        for neighbor, T_edge in adjacency.get(node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, T_acc @ T_edge))

    raise ValueError(
        f"No path found between {link_a} and {link_b} in URDF {urdf_path}. "
        f"Available links: {', '.join(all_links)}"
    )
