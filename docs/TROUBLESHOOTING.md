# Troubleshooting

This page captures the main errors repeatedly seen while running `fruc_ros_utils`.

## Quick Triage Checklist

Run these first before deep debugging:

```bash
# 1) Compose syntax
docker compose -f Docker/ros/docker-compose.yml config -q

# 2) Container status
docker compose -f Docker/ros/docker-compose.yml ps --all

# 3) Core env vars
echo "$BAGS_PATH"
echo "$DISPLAY"
echo "$XAUTHORITY"
```

## Common Errors And Fixes

### 1) `no such service: jazzy` (or `noetic`)

Cause:
- Running `docker compose ... jazzy` from the wrong compose project.

Fix:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags docker compose run --rm jazzy ros2utils --help
```

### 2) Bag path not found in container

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

### 3) GUI/X11 errors (`cannot open display`)

Cause:
- Missing/invalid `DISPLAY` or `XAUTHORITY` pass-through.

Fix:

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
```

Then rerun with preserved env:

```bash
docker compose -f Docker/ros/docker-compose.yml run --rm \
  -e DISPLAY -e XAUTHORITY jazzy ros2utils --help
```

### 4) `register-python-argcomplete was not found`

Symptom:
- `source scripts/enable_autocomplete.sh` prints missing `register-python-argcomplete`.

Fix:

```bash
pip install argcomplete
source scripts/enable_autocomplete.sh
```

### 5) Local CLI import errors (example: `ModuleNotFoundError: No module named 'cv2'`)

Cause:
- Running tools outside the prepared Docker/python environment.

Fix options:
1. Run via Docker (`Docker/ros` noetic/jazzy services).
2. Install package dependencies locally (`pip install .`) in your active environment.

### 6) `--remap entry must be OLD:NEW`

Cause:
- Invalid remap format for `ros2utils convert_to_ros1`, `convert_folder`, or `ros1utils remap_topics`.

Fix:

```bash
ros2utils convert_to_ros1 --bag /bags/in.mcap --remap /old/topic:/new/topic
```

### 7) Output exists errors (`Use --overwrite to replace it`)

Applies to:
- `ros2utils convert_to_ros1`
- `ros1utils crop_pointcloud_fov`
- `ros1utils remap_topics`

Fix:
- Add `--overwrite`, or choose a new output path.

### 8) MCAP conversion edge cases (`KeyError`, `ros_distro` parse failures)

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
- `docs/COMMANDS.md`
