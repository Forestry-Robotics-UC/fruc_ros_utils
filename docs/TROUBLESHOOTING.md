# Troubleshooting

This page captures the main errors repeatedly seen while running `fruc_ros_utils` and `Docker/iKalibr`.

## Quick Triage Checklist

Run these first before deep debugging:

```bash
# 1) Compose syntax
docker compose -f Docker/ros/docker-compose.yml config -q
docker compose -f Docker/iKalibr/docker-compose.yml config -q

# 2) Container status
docker compose -f Docker/ros/docker-compose.yml ps --all
docker compose -f Docker/iKalibr/docker-compose.yml ps --all

# 3) Core env vars
echo "$BAGS_PATH"
echo "$DISPLAY"
echo "$XAUTHORITY"
```

## Common Errors And Fixes

### 1) `BAGS_PATH is not set`

Symptom:
- `Docker/iKalibr/run.sh` exits with: `[iKalibr] BAGS_PATH is not set.`

Fix:

```bash
export BAGS_PATH=/absolute/path/to/bags
cd Docker/iKalibr
./run.sh solver
```

If using `sudo`, preserve env:

```bash
sudo --preserve-env=BAGS_PATH,XAUTHORITY,DISPLAY ./run.sh solver
```

### 2) `no such service: jazzy` (or `noetic`)

Cause:
- Running `docker compose ... jazzy` from the wrong compose project.

Fix:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags docker compose run --rm jazzy ros2utils --help
```

### 3) Bag path not found in container

Symptom:
- File exists on host but CLI says missing path.

Cause:
- Passed host path directly instead of `/bags/...`.

Fix:
- Use host path only for `BAGS_PATH`.
- Use container path in CLI args:

```bash
# host
export BAGS_PATH=/mnt/t7_shield/session

# container-visible
ros2utils info --bag /bags/run_01.mcap
```

### 4) iKalibr solver error: `can not find the ros bag (i.e., DataStream::BagPath)!`

Cause:
- `BagPath` in `Docker/iKalibr/config/ikalibr-config.yaml` does not exist under `/bags`.

Fix:

```bash
# verify mount and bag
docker compose -f Docker/iKalibr/docker-compose.yml run --rm --entrypoint bash solver -lc 'ls -la /bags'
```

Then set `Configor.DataStream.BagPath` to a valid `/bags/...` file.

### 5) `Permission denied` writing iKalibr outputs/config

Typical symptoms:
- Cannot write `Docker/iKalibr/output`
- Permission errors creating output directories inside container

Fix:

```bash
cd Docker/iKalibr
sudo --preserve-env=BAGS_PATH,XAUTHORITY,DISPLAY ./run.sh shell
```

`run.sh` repairs ownership of `config/` and `output/` for the calling user.

### 6) iKalibr `imu-calib` config path mismatch after config cleanup

Symptom:
- Compose references `/workspace/config/tool/config-imu-intri-calib.yaml` but the file was removed.

Current reference:
- `Docker/iKalibr/docker-compose.yml` -> `imu-calib` command `config_path:=/workspace/config/tool/config-imu-intri-calib.yaml`

Fix options:
1. Restore the expected file path under `Docker/iKalibr/config/tool/`.
2. Update `imu-calib` command/config path to the new intended config location and validate with:

```bash
docker compose -f Docker/iKalibr/docker-compose.yml config -q
```

### 7) GUI/X11 errors (`cannot open display`)

Cause:
- Missing/invalid `DISPLAY` or `XAUTHORITY` pass-through.

Fix:

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
```

Then rerun with preserved env:

```bash
sudo --preserve-env=BAGS_PATH,XAUTHORITY,DISPLAY ./run.sh shell
```

### 8) GPU path issues (`nvidia-smi` unavailable or unhealthy)

Cause:
- Host NVIDIA stack not healthy.

Fix:

```bash
nvidia-smi
```

If unavailable, use CPU path:

```bash
cd Docker/iKalibr
./run.sh solver
```

### 9) `register-python-argcomplete was not found`

Symptom:
- `source scripts/enable_autocomplete.sh` prints missing `register-python-argcomplete`.

Fix:

```bash
pip install argcomplete
source scripts/enable_autocomplete.sh
```

### 10) Local CLI import errors (example: `ModuleNotFoundError: No module named 'cv2'`)

Cause:
- Running tools outside the prepared Docker/python environment.

Fix options:
1. Run via Docker (`Docker/ros` noetic/jazzy services).
2. Install package dependencies locally (`pip install .`) in your active environment.

### 11) `--remap entry must be OLD:NEW`

Cause:
- Invalid remap format for `ros2utils convert_to_ros1` or `convert_folder`.

Fix:

```bash
ros2utils convert_to_ros1 --bag /bags/in.mcap --remap /old/topic:/new/topic
```

### 12) Output exists errors (`Use --overwrite to replace it`)

Applies to:
- `ros2utils convert_to_ros1`
- `scripts/repack_pointcloud_for_ikalibr.py`
- `scripts/crop_pointcloud_fov.py`

Fix:
- Add `--overwrite`, or choose a new output path.

### 13) MCAP conversion edge cases (`KeyError`, `ros_distro` parse failures)

Cause:
- Corrupt/non-standard MCAP schema metadata.

Fix pattern:
1. Start with a minimal include list (`--include-topic ...`) for critical topics.
2. Exclude known problematic topics (`--exclude-topic ...`).
3. Retry conversion and inspect logs.

Example:

```bash
ros2utils convert_to_ros1 \
  --bag /bags/session.mcap \
  --out /bags/session_ros1.bag \
  --include-topic /imu/data /ouster/points /camera/color/image_raw
```

## Related Docs

- `docs/DOCKER.md`
- `docs/IKALIBR_DOCKER.md`
- `docs/COMMANDS.md`
