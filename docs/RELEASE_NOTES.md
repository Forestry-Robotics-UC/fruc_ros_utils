# Release Notes

## v0.1.0

This is the first release of `fruc_ros_utils` as a single published utility bundle.

### Main Features

- `ros2utils` for ROS 2 bag inspection, topic listing, duration checks, and ROS 2 -> ROS 1 conversion.
- `ros1utils` and `bagutils` for ROS 1 bag processing, metadata export, navsat tools, and image-based workflows.
- `ros1utils remap_topics` for renaming topics in ROS 1 bags.
- `extract_images_manifest` and `images_manifest_to_bag` for round-tripping ROS 1 image topics through PNG manifests.
- `mapir_ndvi` and the `mapir_ndvi_node.py` script for MAPIR NDVI workflows.
- `convert_bags_per_duration.py` for chunked ROS 2 -> ROS 1 conversion.
- Dockerized ROS tooling under `Docker/ros` for Jazzy and Noetic workflows.

### Notes

- The old calibration-specific code and documentation were removed from this repo.
- The supported CLI surface now centers on `bagutils`, `ros1utils`, and `ros2utils`.
- Package version and release tag are aligned at `0.1.0`.
