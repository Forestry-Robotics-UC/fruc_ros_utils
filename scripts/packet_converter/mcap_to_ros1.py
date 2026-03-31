#!/usr/bin/env python3
"""
Convert MCAP rosbag2 bags → ROS1 .bag files using fruc_ros_utils.

Supports two modes:
  1. Folder mode (primary):  Pass a rosbag2 directory (contains metadata.yaml).
     Produces one .bag per MCAP file listed in metadata.yaml.
  2. Single-file mode:  Pass a single .mcap file.
     Wraps it in a temporary rosbag2 directory and converts.

Usage:
  # Folder — converts every MCAP listed in metadata.yaml that exists on disk
  python3 mcap_to_ros1.py /path/to/bag_dir /path/to/output_dir --fruc-path ...

  # Single file
  python3 mcap_to_ros1.py /path/to/file.mcap /path/to/output_dir --fruc-path ...
"""

import copy
import os
import sys
import argparse
import tempfile
from pathlib import Path

import yaml


# ------------------------------------------------------------------ helpers --

def _make_single_file_metadata(master_meta: dict, file_entry: dict, mcap_name: str) -> dict:
    """
    Build a metadata.yaml dict for a temporary rosbag2 directory that contains
    a single MCAP file.  Keeps the topic definitions from the master but
    adjusts file paths, message_count, duration, and starting_time.
    """
    meta = copy.deepcopy(master_meta)
    info = meta["rosbag2_bagfile_information"]

    # rosbags 0.9.x only supports metadata version ≤ 8; Jazzy records version 9
    # but the format is compatible — just cap the version number.
    if info.get("version", 0) > 8:
        info["version"] = 8

    # Strip type_description_hash from topic metadata (version 8 doesn't have it)
    for tc in info.get("topics_with_message_count", []):
        tc.get("topic_metadata", {}).pop("type_description_hash", None)

    # Point to the single file
    info["relative_file_paths"] = [mcap_name]
    info["files"] = [
        {
            "path": mcap_name,
            "starting_time": copy.deepcopy(file_entry["starting_time"]),
            "duration": copy.deepcopy(file_entry["duration"]),
            "message_count": file_entry["message_count"],
        }
    ]

    # Update aggregate stats to match single file
    info["message_count"] = file_entry["message_count"]
    info["starting_time"] = copy.deepcopy(file_entry["starting_time"])
    info["duration"] = copy.deepcopy(file_entry["duration"])

    return meta


def _get_exclude_topics(meta: dict) -> list:
    """
    Return a list of topics whose message types are NOT in the standard
    rosbags typestore, so rosbags.convert can skip them instead of crashing.
    """
    _STANDARD_PREFIXES = (
        "sensor_msgs/", "std_msgs/", "geometry_msgs/", "nav_msgs/",
        "builtin_interfaces/", "rosgraph_msgs/", "tf2_msgs/",
        "diagnostic_msgs/", "visualization_msgs/", "shape_msgs/",
        "actionlib_msgs/", "stereo_msgs/", "trajectory_msgs/",
    )
    exclude = []
    info = meta.get("rosbag2_bagfile_information", {})
    for tc in info.get("topics_with_message_count", []):
        tm = tc.get("topic_metadata", {})
        msg_type = tm.get("type", "")
        if not any(msg_type.startswith(p) for p in _STANDARD_PREFIXES):
            topic = tm.get("name", "")
            if topic:
                exclude.append(topic)
    return exclude


def _convert_single_mcap(
    bag_dir: Path,
    mcap_name: str,
    master_meta: dict,
    file_entry: dict,
    output_bag: Path,
):
    """
    Create a temporary rosbag2 directory for one MCAP, then convert to ROS1
    using rosbags.convert directly (bypasses ros2utils API issues).
    """
    from rosbags.convert import convert as rosbags_convert

    mcap_path = bag_dir / mcap_name

    with tempfile.TemporaryDirectory(prefix="ros2bag_") as tmpdir:
        tmp = Path(tmpdir)

        # Symlink the MCAP file into the temp directory
        (tmp / mcap_name).symlink_to(mcap_path)

        # Write a metadata.yaml that references only this file
        single_meta = _make_single_file_metadata(master_meta, file_entry, mcap_name)
        (tmp / "metadata.yaml").write_text(yaml.dump(single_meta, default_flow_style=False))

        # Auto-detect topics with custom types that would crash rosbags.convert
        exclude = _get_exclude_topics(single_meta)
        if exclude:
            print(f"[INFO] Excluding {len(exclude)} topics with custom types: {', '.join(exclude)}")

        print(f"[INFO] Converting: {mcap_name}  →  {output_bag.name}")
        print(f"       Messages: {file_entry['message_count']}")

        # Use rosbags.convert directly — handles ROS2↔ROS1 typestore conversion
        rosbags_convert(tmp, output_bag, exclude_topics=exclude)
        print(f"[OK]   {output_bag.name}  ({output_bag.stat().st_size / 1e6:.1f} MB)")


def _generate_metadata_for_single_file(mcap_path: Path) -> dict:
    """
    Generate a minimal metadata.yaml dict for a standalone MCAP file that has
    no accompanying metadata.yaml.  Uses rosbag2_py to introspect topics.
    """
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(mcap_path), storage_id="mcap")
    converter_opts = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter_opts)

    topics = reader.get_all_topics_and_types()
    topic_entries = []
    for t in topics:
        topic_entries.append({
            "topic_metadata": {
                "name": t.name,
                "type": t.type,
                "serialization_format": t.serialization_format if t.serialization_format else "cdr",
                "offered_qos_profiles": "",
                "type_description_hash": "",
            },
            "message_count": 0,
        })

    # Read all messages to get timing and counts
    first_ts = last_ts = None
    msg_count = 0
    topic_counts = {}
    while reader.has_next():
        topic_name, _data, ts = reader.read_next()
        msg_count += 1
        topic_counts[topic_name] = topic_counts.get(topic_name, 0) + 1
        if first_ts is None:
            first_ts = ts
        last_ts = ts

    # Update per-topic counts
    for entry in topic_entries:
        tname = entry["topic_metadata"]["name"]
        entry["message_count"] = topic_counts.get(tname, 0)

    duration_ns = (last_ts - first_ts) if (first_ts and last_ts) else 0

    mcap_name = mcap_path.name
    return {
        "rosbag2_bagfile_information": {
            "version": 8,
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": duration_ns},
            "starting_time": {"nanoseconds_since_epoch": first_ts or 0},
            "message_count": msg_count,
            "topics_with_message_count": topic_entries,
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [mcap_name],
            "files": [
                {
                    "path": mcap_name,
                    "starting_time": {"nanoseconds_since_epoch": first_ts or 0},
                    "duration": {"nanoseconds": duration_ns},
                    "message_count": msg_count,
                }
            ],
            "custom_data": None,
        }
    }


# -------------------------------------------------------------------- main --

def main():
    parser = argparse.ArgumentParser(
        description="Convert MCAP rosbag2 bags → ROS1 .bag files via fruc_ros_utils"
    )
    parser.add_argument(
        "input_path",
        help="Rosbag2 directory (with metadata.yaml) OR single .mcap file",
    )
    parser.add_argument(
        "output_dir",
        help="Directory to write .bag files into",
    )
    parser.add_argument(
        "--fruc-path", default="../fruc_ros_utils",
        help="Path to fruc_ros_utils repo",
    )
    parser.add_argument(
        "--only", nargs="*", default=None,
        help="Only convert these MCAP filenames (e.g. calib_test__0.mcap calib_test__1.mcap)",
    )

    args = parser.parse_args()

    input_path = Path(args.input_path).resolve()
    output_dir = Path(args.output_dir).resolve()
    fruc_path = Path(args.fruc_path).resolve()

    if not input_path.exists():
        print(f"[ERROR] Input not found: {input_path}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    # ---- Determine mode: folder vs single file ----

    if input_path.is_dir():
        # FOLDER MODE — read metadata.yaml, convert each MCAP
        meta_file = input_path / "metadata.yaml"
        if not meta_file.exists():
            print(f"[ERROR] No metadata.yaml in {input_path}", file=sys.stderr)
            return 1

        with open(meta_file) as f:
            master_meta = yaml.safe_load(f)

        info = master_meta["rosbag2_bagfile_information"]
        all_files = info.get("files", [])

        # Filter to files that exist on disk
        present = []
        for fe in all_files:
            mcap_name = fe["path"]
            if (input_path / mcap_name).exists():
                present.append(fe)
            else:
                print(f"[SKIP] Not on disk: {mcap_name}")

        # Apply --only filter
        if args.only:
            only_set = set(args.only)
            present = [fe for fe in present if fe["path"] in only_set]

        if not present:
            print("[ERROR] No MCAP files to convert", file=sys.stderr)
            return 1

        print(f"[INFO] Folder mode: {input_path}")
        print(f"[INFO] Converting {len(present)}/{len(all_files)} MCAP files → {output_dir}")
        print()

        # When a rosbag2 dir has only 1 file (e.g. processed output), use the
        # directory name for the bag stem instead of the inner mcap filename
        # (which has an ugly _0 suffix like "test__0_processed.mcap_0.mcap").
        use_dir_stem = (len(present) == 1 and len(all_files) == 1)

        ok = 0
        fail = 0
        for fe in present:
            mcap_name = fe["path"]

            if use_dir_stem:
                # Use directory name: "test__0_processed.mcap" → "test__0_processed.bag"
                bag_name = Path(input_path.name).stem + ".bag"
            else:
                bag_name = Path(mcap_name).stem + ".bag"

            out_bag = output_dir / bag_name

            try:
                _convert_single_mcap(
                    input_path, mcap_name, master_meta, fe, out_bag
                )
                ok += 1
            except Exception as e:
                fail += 1
                print(f"[FAIL] {mcap_name}: {e}", file=sys.stderr)
                import traceback
                traceback.print_exc()

        print()
        print(f"[DONE] {ok} converted, {fail} failed out of {len(present)}")
        return 1 if fail > 0 else 0

    else:
        # SINGLE FILE MODE — wrap in temp rosbag2 dir
        if not input_path.suffix.lower() == ".mcap":
            print(f"[ERROR] Expected .mcap file, got: {input_path}", file=sys.stderr)
            return 1

        print(f"[INFO] Single-file mode: {input_path.name}")

        # Check if a metadata.yaml exists in the parent (part of a bigger bag)
        parent_meta = input_path.parent / "metadata.yaml"
        mcap_name = input_path.name

        if parent_meta.exists():
            with open(parent_meta) as f:
                master_meta = yaml.safe_load(f)

            # Find this file's entry in the metadata
            info = master_meta["rosbag2_bagfile_information"]
            file_entry = None
            for fe in info.get("files", []):
                if fe["path"] == mcap_name:
                    file_entry = fe
                    break

            if file_entry is None:
                print(f"[WARN] {mcap_name} not in parent metadata.yaml, generating metadata")
                master_meta = _generate_metadata_for_single_file(input_path)
                file_entry = master_meta["rosbag2_bagfile_information"]["files"][0]
        else:
            print(f"[INFO] No parent metadata.yaml, generating from MCAP introspection")
            master_meta = _generate_metadata_for_single_file(input_path)
            file_entry = master_meta["rosbag2_bagfile_information"]["files"][0]

        bag_name = input_path.stem + ".bag"
        out_bag = output_dir / bag_name

        try:
            _convert_single_mcap(
                input_path.parent, mcap_name, master_meta, file_entry, out_bag
            )
            print(f"\n[DONE] 1 converted, 0 failed")
            return 0
        except Exception as e:
            print(f"[FAIL] {mcap_name}: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return 1


if __name__ == "__main__":
    sys.exit(main())
