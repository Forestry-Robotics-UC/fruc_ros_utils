# Docker Guide

Use this document when you are not sure which compose file to run, where paths should point, or whether a command belongs on the host or inside a container.

## Compose Projects

This repo has multiple Docker compose projects:

| Compose file | Purpose | Services |
| --- | --- | --- |
| `Docker/ros/docker-compose.yml` | Run `ros2utils` and `ros1utils` | `jazzy`, `noetic` |
| `Docker/iKalibr/docker-compose.yml` | Run iKalibr tools | `solver`, `solver-cuda`, `imu-calib`, `shell` |
| `Docker/packet_converter/docker-compose.yml` | Packet conversion workflow | packet converter services only |

If you need `ros2utils convert_to_ros1`, use `Docker/ros/docker-compose.yml`.

## How The CLI Gets Installed

The ROS utility Docker images install this repo with `pip install .`, so the command names come from `pyproject.toml`:

- `bagutils`
- `ros1utils`
- `ros2utils`

This is the same command surface that the catkin package exposes locally through `setup.py` and installed wrapper scripts. The goal is that Docker and non-Docker usage share the same CLI names.

## iKalibr GPU Builds

The base iKalibr image may contain CPU-only `colmap` / `glomap` binaries even when the container itself can see your GPU.

Two optional source rebuild paths are available:
- `IKALIBR_REBUILD_CUDA=1`: rebuild Ceres and iKalibr with CUDA support
- `IKALIBR_REBUILD_SFM_CUDA=1`: rebuild COLMAP, PoseLib, and GLOMAP with CUDA support

Rebuild only the SfM stack:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/iKalibr
IKALIBR_REBUILD_SFM_CUDA=1 COLMAP_CUDA_ARCHITECTURES=all-major docker compose build --no-cache shell solver-cuda
```

Rebuild the full GPU pipeline:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/iKalibr
IKALIBR_REBUILD_CUDA=1 IKALIBR_REBUILD_SFM_CUDA=1 COLMAP_CUDA_ARCHITECTURES=all-major docker compose build --no-cache shell solver-cuda
```

Optional build pins:
- `COLMAP_GIT_REF`
- `GLOMAP_GIT_REF`
- `POSELIB_GIT_REF`
- `COLMAP_CUDA_ARCHITECTURES`

Current defaults:
- `COLMAP_GIT_REF=3.12.6`
- `GLOMAP_GIT_REF=1.2.0`
- `POSELIB_GIT_REF=master`

Those defaults are pinned to stable releases instead of `main` because the base iKalibr image ships an older CUDA toolchain.
For Docker image builds, prefer `all-major` or an explicit architecture list instead of `native`, because the build container cannot see your host GPU during `docker compose build`.

## Execution Model

Use this mental model:
- host shell: run `docker compose ...`
- inside `jazzy`: run `ros2utils ...`
- inside `noetic`: run `ros1utils ...`

Do not run `docker compose run --rm jazzy ...` from inside an already-running container shell.

## Path Mapping

The ROS compose stack mounts your host bag root at `/bags` inside the container.

Example:
- host path: `/mnt/t7_shield/my_session`
- env var: `BAGS_PATH=/mnt/t7_shield/my_session`
- container-visible root: `/bags`
- file inside container: `/bags/run_01.mcap`

That means:
- host shell sets `BAGS_PATH`
- CLI arguments passed to `ros2utils` or `ros1utils` should use `/bags/...`

Wrong:

```bash
docker compose run --rm jazzy ros2utils convert_to_ros1 --bag run_01.mcap
```

Right:

```bash
docker compose run --rm jazzy ros2utils convert_to_ros1 --bag /bags/run_01.mcap
```

## Quick Start

Build the utility images:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose build noetic jazzy
```

The `jazzy` image installs `rosbags` from PyPI by default:
- default install spec: `rosbags==0.10.11`
- override for A/B testing: set `ROSBAGS_INSTALL_SPEC`

Examples:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose build --no-cache jazzy
BAGS_PATH=/path/to/bags_root ROSBAGS_INSTALL_SPEC='rosbags==0.10.11' docker compose build --no-cache jazzy
BAGS_PATH=/path/to/bags_root ROSBAGS_INSTALL_SPEC='rosbags==0.10.4' docker compose build --no-cache jazzy
```

Check that the CLIs are available:

```bash
BAGS_PATH=/path/to/bags_root docker compose run --rm jazzy ros2utils --help
BAGS_PATH=/path/to/bags_root docker compose run --rm noetic ros1utils --help
```

## Typical Host-Shell Commands

For Jazzy -> Noetic bag conversion, use the offline flow in `jazzy` first:
- `ros2utils info` to inspect the bag
- `ros2utils convert_to_ros1` to write a ROS 1 `.bag`
- only use a live bridge and re-record workflow if custom types prevent offline conversion

Inspect a ROS 2 bag:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose run --rm jazzy \
  ros2utils info --bag /bags/session/run_01.mcap
```

Convert one ROS 2 bag to ROS 1:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose run --rm jazzy \
  ros2utils convert_to_ros1 \
    --bag /bags/session/run_01.mcap \
    --out /bags/session/run_01_ros1.bag
```

Convert with time-based chunking:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose run --rm jazzy \
  ros2utils convert_to_ros1 \
    --bag /bags/session/run_01.mcap \
    --out /bags/session/run_01_ros1/ \
    --split-duration 10m
```

If you prefer the older duration-wrapper workflow, it now delegates to the same `ros2utils convert_to_ros1 --split-duration` implementation instead of maintaining a second bag-splitting path.

Run a ROS 1 command:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose run --rm noetic \
  ros1utils calculate_bag_duration --in /bags/session/run_01_ros1.bag --total
```

## If You Are Already Inside a Container

Do not prefix commands with `docker compose`.

Inside `jazzy`:

```bash
ros2utils list_topics --bag /bags/session/run_01.mcap
ros2utils convert_to_ros1 --bag /bags/session/run_01.mcap --out /bags/session/run_01_ros1.bag
```

Inside `noetic`:

```bash
ros1utils calculate_bag_duration --in /bags/session/run_01_ros1.bag --total
python3 /workspace/fruc_ros_utils/src/bag/sync_audit_ros1.py /bags/session/run_01_ros1.bag
```

## Common Failure Cases

### `no such service: jazzy`

Cause:
- you are in the wrong compose project, usually `Docker/iKalibr`

Wrong:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/iKalibr
BAGS_PATH=/mnt/t7_shield/my_session docker compose run --rm jazzy ...
```

Right:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/mnt/t7_shield/my_session docker compose run --rm jazzy ...
```

### File not found inside container

Cause:
- you passed a host path or bare filename to the CLI instead of `/bags/...`

Wrong:

```bash
ros2utils convert_to_ros1 --bag /mnt/t7_shield/my_session/run_01.mcap
```

Right:

```bash
ros2utils convert_to_ros1 --bag /bags/run_01.mcap
```

### Docker is unavailable

If you cannot run Docker in the current session, use:
- a machine where Docker is available, or
- a shell that already has the correct ROS environment

Environment expectations:
- `ros2utils` expects ROS 2 Jazzy
- `ros1utils` expects ROS 1 Noetic
