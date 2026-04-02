# iKalibr Docker Guide

This guide covers the `Docker/iKalibr` stack, with practical fixes for the most common runtime failures.

## 1. Prerequisites

- Docker Engine + Docker Compose plugin
- ROS bag data directory on host
- X11 available on host (for viewer/shell workflows)
- Optional but recommended for GPU services: NVIDIA driver + `nvidia-smi`

Quick host checks:

```bash
docker --version
docker compose version
nvidia-smi || true
```

## 2. First-Time Setup

From repo root:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils
chmod +x Docker/iKalibr/run.sh
./scripts/apply_compose_memory_limits.sh 70
```

Set bags path (host folder mounted as `/bags` in container):

```bash
export BAGS_PATH=/absolute/path/to/your/bags
```

Build image:

```bash
cd Docker/iKalibr
docker compose build
```

## 3. Recommended Run Commands

From `Docker/iKalibr`:

CPU solver:

```bash
BAGS_PATH=/absolute/path/to/bags ./run.sh solver
```

GPU solver:

```bash
BAGS_PATH=/absolute/path/to/bags ./run.sh solver-cuda
```

IMU intrinsics:

```bash
BAGS_PATH=/absolute/path/to/bags ./run.sh imu-calib
```

Interactive shell:

```bash
BAGS_PATH=/absolute/path/to/bags ./run.sh shell
```

## 4. Path and Config Mapping

Host to container mounts (from `docker-compose.yml`):

- `${BAGS_PATH}` -> `/bags` (read-only)
- `Docker/iKalibr/config` -> `/workspace/config` (read-write)
- `Docker/iKalibr/output` -> `/workspace/output` (read-write)

Default config consumed by solver:

- `/workspace/config/ikalibr-config.yaml`

## 5. Frequent Issues and Fixes

### A) `BAGS_PATH is not set`

Set it in the same shell that runs `run.sh`:

```bash
export BAGS_PATH=/absolute/path/to/bags
./run.sh solver
```

### B) Permission denied writing `output/` or editing `config/`

Run once with sudo so helper can repair ownership:

```bash
sudo --preserve-env=BAGS_PATH,XAUTHORITY,DISPLAY ./run.sh shell
```

Then rerun as normal user.

### C) GPU service requested but CUDA path fails

Validate host GPU first:

```bash
nvidia-smi
```

If unavailable, run CPU solver:

```bash
./run.sh solver
```

### D) X11/GUI issues (`cannot open display`)

Check these vars:

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
```

Then retry with env preserved:

```bash
sudo --preserve-env=BAGS_PATH,XAUTHORITY,DISPLAY ./run.sh shell
```

### E) Out-of-memory during build/run

Regenerate compose memory limits:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils
./scripts/apply_compose_memory_limits.sh 75
```

Then rebuild:

```bash
cd Docker/iKalibr
docker compose build
```

### F) Solver starts but fails to find expected bag/config paths

Inside shell, verify mounts:

```bash
ls -la /bags
ls -la /workspace/config
ls -la /workspace/output
```

## 6. Optional CUDA/SfM Rebuilds

For advanced rebuilds, set build args before `docker compose build`:

```bash
cd Docker/iKalibr
IKALIBR_REBUILD_SFM_CUDA=1 COLMAP_CUDA_ARCHITECTURES=all-major docker compose build --no-cache shell solver-cuda
```

Full CUDA pipeline rebuild:

```bash
cd Docker/iKalibr
IKALIBR_REBUILD_CUDA=1 IKALIBR_REBUILD_SFM_CUDA=1 COLMAP_CUDA_ARCHITECTURES=all-major docker compose build --no-cache shell solver-cuda
```

## 7. Minimal Debug Workflow

1. `./scripts/apply_compose_memory_limits.sh 70`
2. `BAGS_PATH=... ./Docker/iKalibr/run.sh shell`
3. Validate `/bags` and `/workspace/config`
4. Run solver command manually inside shell
5. Inspect `/workspace/output`

## 8. Camera Calibration

Camera intrinsics are defined in a single file:

- `Docker/iKalibr/config/cam_calib.yaml`

Use this path when updating camera calibration defaults.
