#!/usr/bin/env python3
"""
Generate iKalibr spat-temp-priori.yaml from a URDF/xacro file.

Parses the kinematic tree and computes transforms between sensor frames,
then writes the prior file that iKalibr uses for initial extrinsic estimates.

Usage:
  python3 urdf_to_ikalibr_priori.py \
    --urdf sensor_rig.urdf.xacro \
    --map /imu/data:imu_link /ouster/points:os_lidar /camera/color/image_raw:camera_color_optical_frame \
    --ref /imu/data \
    --out spat-temp-priori.yaml
python3 urdf_to_ikalibr_priori.py \
    --urdf /mnt/bumble/work_utils/fruc_ros_utils/src/Docker/iKalibr/config/sensor_rig.urdf.xacro \
   --map /imu/data:imu_link  /ouster/points:os_lidar /camera/color/image_raw:camera_color_optical_frame \
   --ref /imu/data \
   --out /mnt/bumble/work_utils/fruc_ros_utils/src/Docker/iKalibr/config/spat-temp-priori.yaml

The --map argument defines topic:frame mappings. The --ref argument is the
reference IMU topic (must be one of the mapped topics). The script computes
transforms between all sensor pairs relative to the reference and writes
the output YAML.
"""

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict


# ──────────────────────────────────────────────────────────────────────────
# Quaternion / SE(3) helpers  (no numpy needed)
# ──────────────────────────────────────────────────────────────────────────

def rpy_to_quat(roll, pitch, yaw):
    """RPY (XYZ intrinsic = ZYX extrinsic) → quaternion (qx, qy, qz, qw)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(q1, q2):
    """Hamilton product q1*q2.  Format: (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    """Rotate 3-vector v by unit quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return r[:3]


IDENTITY_TF = ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0))


def tf_compose(t1, t2):
    """T1 * T2.  Each transform is (quat, pos)."""
    q1, p1 = t1
    q2, p2 = t2
    q = quat_multiply(q1, q2)
    r = quat_rotate(q1, p2)
    return (q, (p1[0] + r[0], p1[1] + r[1], p1[2] + r[2]))


def tf_inverse(t):
    q, p = t
    qi = quat_conjugate(q)
    pi = quat_rotate(qi, p)
    return (qi, (-pi[0], -pi[1], -pi[2]))


# ──────────────────────────────────────────────────────────────────────────
# URDF / xacro parser
# ──────────────────────────────────────────────────────────────────────────

def safe_eval(expr_str):
    """Evaluate simple xacro math expressions like '${pi/2}' or '${1.58 + PI}'."""
    s = expr_str.strip()
    if s.startswith("${") and s.endswith("}"):
        s = s[2:-1]
    s = re.sub(r"\bPI\b", str(math.pi), s)
    s = re.sub(r"\bpi\b", str(math.pi), s)
    try:
        return float(eval(s, {"__builtins__": {}}, {"pi": math.pi, "PI": math.pi}))
    except Exception:
        return float(s)


def parse_origin(elem):
    """Parse <origin xyz="..." rpy="..." /> → (xyz_tuple, rpy_tuple)."""
    xyz = (0.0, 0.0, 0.0)
    rpy = (0.0, 0.0, 0.0)
    if elem is not None:
        xyz_s = elem.get("xyz", "0 0 0")
        rpy_s = elem.get("rpy", "0 0 0")
        xyz = tuple(safe_eval(v) for v in xyz_s.split())
        rpy = tuple(safe_eval(v) for v in rpy_s.split())
    return xyz, rpy


def parse_urdf_tree(urdf_path):
    """Parse URDF and return kinematic tree: {child: (parent, transform)}.

    Tries xacro first; falls back to raw XML (handles ${} expressions).
    """
    root = None
    # attempt xacro
    try:
        import xacro
        doc = xacro.process_file(str(urdf_path))
        root = ET.fromstring(doc.toxml())
    except Exception:
        pass

    if root is None:
        # raw XML parse – strip xacro namespace noise if needed
        with open(urdf_path, "r") as f:
            xml_str = f.read()
        # Remove xacro namespace-prefixed elements that are not joints/links
        xml_str = re.sub(r"<xacro:include[^/]*/\s*>", "", xml_str)
        xml_str = re.sub(r"<xacro:property[^/]*/\s*>", "", xml_str)
        # Resolve ${expr} in attribute values (only in origin xyz/rpy attrs)
        def _resolve(m):
            try:
                return str(safe_eval(m.group(0)))
            except (ValueError, NameError, SyntaxError):
                # Non-numeric expression (e.g. ${mesh_prefix}...) – leave as-is
                return m.group(0)
        xml_str = re.sub(r"\$\{[^}]+\}", _resolve, xml_str)
        root = ET.fromstring(xml_str)

    tree = {}
    for joint in root.iter("joint"):
        jtype = joint.get("type", "")
        if jtype != "fixed":
            continue
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            continue
        parent = parent_elem.get("link")
        child = child_elem.get("link")
        origin = joint.find("origin")
        xyz, rpy = parse_origin(origin)
        q = rpy_to_quat(*rpy)
        # URDF convention: joint gives pose of child in parent frame
        # i.e. p_parent = R * p_child + t  →  T_{child→parent}
        tree[child] = (parent, (q, xyz))

    return tree


def link_to_root(tree, link):
    """Compute T_{link→root} by walking parent chain."""
    result = IDENTITY_TF
    current = link
    while current in tree:
        parent, tf = tree[current]
        # tf = T_{current→parent}
        result = tf_compose(tf, result)
        current = parent
    return result, current


def compute_transform(tree, from_link, to_link):
    """Compute T_{from→to}: p_to = T * p_from."""
    if from_link == to_link:
        return IDENTITY_TF
    t_from, root_a = link_to_root(tree, from_link)
    t_to, root_b = link_to_root(tree, to_link)
    if root_a != root_b:
        raise ValueError(
            "Links '{}' and '{}' are not in the same tree (roots: {}, {})".format(
                from_link, to_link, root_a, root_b
            )
        )
    return tf_compose(tf_inverse(t_to), t_from)


# ──────────────────────────────────────────────────────────────────────────
# YAML writer
# ──────────────────────────────────────────────────────────────────────────

def write_priori_yaml(pairs, out_path):
    """Write iKalibr spat-temp-priori.yaml.

    pairs: list of (topic_first, topic_second, quat, pos)
    """
    lines = [
        "# Auto-generated from URDF by urdf_to_ikalibr_priori.py",
        "SpatialTemporalPriori:",
        "  SO3_Sen1ToSen2:",
    ]
    for first, second, q, p in pairs:
        lines += [
            "    - key:",
            '        first: "{}"'.format(first),
            '        second: "{}"'.format(second),
            "      value:",
            "        qx: {:.15g}".format(q[0]),
            "        qy: {:.15g}".format(q[1]),
            "        qz: {:.15g}".format(q[2]),
            "        qw: {:.15g}".format(q[3]),
        ]
    lines.append("  POS_Sen1InSen2:")
    for first, second, q, p in pairs:
        lines += [
            "    - key:",
            '        first: "{}"'.format(first),
            '        second: "{}"'.format(second),
            "      value:",
            "        r0c0: {:.15g}".format(p[0]),
            "        r1c0: {:.15g}".format(p[1]),
            "        r2c0: {:.15g}".format(p[2]),
        ]
    lines += [
        "  TO_Sen1ToSen2:",
        "  # time offsets (none from URDF)",
        "  RS_READOUT:",
        "  # readout times (none from URDF)",
    ]

    text = "\n".join(lines) + "\n"
    with open(out_path, "w") as f:
        f.write(text)
    return text


# ──────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────

def make_parser():
    p = argparse.ArgumentParser(
        description="Generate iKalibr spat-temp-priori.yaml from URDF"
    )
    p.add_argument("--urdf", required=True, help="Path to URDF or xacro file")
    p.add_argument(
        "--map",
        nargs="+",
        metavar="TOPIC:FRAME",
        default=[
            "/imu/data:imu_link",
            "/ouster/points:os_lidar",
            "/camera/color/image_raw:camera_color_optical_frame",
        ],
        help="Topic-to-URDF-frame mappings (default: ouster + realsense)",
    )
    p.add_argument(
        "--ref",
        default="/imu/data",
        help="Reference IMU topic (must appear in --map)",
    )
    p.add_argument(
        "--out",
        default="spat-temp-priori.yaml",
        help="Output YAML path",
    )
    p.add_argument("--verbose", "-v", action="store_true")
    return p


def main():
    args = make_parser().parse_args()

    # Parse topic:frame mappings
    topic_to_frame = {}
    for m in args.map:
        if ":" not in m:
            print("ERROR: mapping must be topic:frame, got '{}'".format(m))
            return 2
        topic, frame = m.split(":", 1)
        topic_to_frame[topic] = frame

    if args.ref not in topic_to_frame:
        print("ERROR: --ref '{}' not in --map topics".format(args.ref))
        return 2

    ref_frame = topic_to_frame[args.ref]

    # Parse URDF
    print("Parsing URDF: {}".format(args.urdf))
    tree = parse_urdf_tree(args.urdf)
    print("  Found {} fixed joints".format(len(tree)))

    all_links = set(tree.keys())
    for child, (parent, _) in tree.items():
        all_links.add(parent)

    for topic, frame in topic_to_frame.items():
        if frame not in all_links:
            print("WARNING: frame '{}' (topic '{}') not found in URDF links: {}".format(
                frame, topic, sorted(all_links)
            ))

    # Compute transforms between all sensors and the reference
    pairs = []
    ref_topic = args.ref
    for topic, frame in sorted(topic_to_frame.items()):
        if topic == ref_topic:
            continue
        ref_f = topic_to_frame[ref_topic]
        # T_{sensor→ref}: transforms points from sensor frame to ref frame
        # iKalibr wants: SO3_first_to_second, POS_first_in_second
        # first = this sensor, second = ref
        tf = compute_transform(tree, frame, ref_f)
        q, p = tf
        pairs.append((topic, ref_topic, q, p))

        if args.verbose:
            print("  {} ({}) → {} ({})".format(topic, frame, ref_topic, ref_f))
            print("    quat: ({:.6f}, {:.6f}, {:.6f}, {:.6f})".format(*q))
            print("    pos:  ({:.6f}, {:.6f}, {:.6f})".format(*p))

    if not pairs:
        print("No sensor pairs to write (only reference sensor found)")
        return 1

    # Write output
    text = write_priori_yaml(pairs, args.out)
    print("\nWrote {} sensor pair(s) to: {}".format(len(pairs), args.out))
    if args.verbose:
        print("\n" + text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
