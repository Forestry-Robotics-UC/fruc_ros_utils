# Command Reference

Use this document when you already know which tool you want and need the command shape, main flags, and a working example.

## Tool Map

| Tool | Best for |
| --- | --- |
| `bagutils` | dispatcher entrypoint and legacy convenience dispatch |
| `ros2utils` | ROS 2 bag inspection and ROS 2 -> ROS 1 conversion |
| `ros1utils` | ROS 1 bag cleanup, metadata export, navsat, frame fixes |
| `scripts/` | standalone helpers and batch wrappers |

## CLI Autocomplete

Autocomplete is available for `bagutils`, `ros1utils`, `ros2utils`, and argparse-based wrapper scripts.

Enable it for the current shell:

```bash
source scripts/enable_autocomplete.sh
```

Manual registration:

```bash
eval "$(register-python-argcomplete bagutils)"
eval "$(register-python-argcomplete ros1utils)"
eval "$(register-python-argcomplete ros2utils)"
```

## `bagutils`

`bagutils` forwards to the ROS-specific CLIs.

```bash
bagutils ros1 <ros1utils args...>
bagutils ros2 <ros2utils args...>
```

Legacy convenience dispatch:

```bash
bagutils convert_to_ros1 --bag /path/to/bag.mcap
bagutils calculate_bag_duration --in /path/to/bag.bag
```

## `ros2utils`

Global form:

```bash
ros2utils [--log-level LEVEL] [--log-file PATH] <command> ...
```

### Inspect a ROS 2 bag

Offline conversion should start here. Use `info` first to confirm storage (`mcap` vs `sqlite3`), topic types, and whether custom ROS 2 messages will need filtering.

Summarize bag metadata and per-topic counts:

```bash
ros2utils info --bag /path/to/bag.mcap
```

List topics and message types:

```bash
ros2utils list_topics --bag /path/to/bag.mcap
```

Measure duration:

```bash
ros2utils duration --bag /path/to/bag.mcap
```

Extract embedded JSON payloads:

```bash
ros2utils extract_json --bag /path/to/bag.mcap
ros2utils extract_json --bag /path/to/bag.mcap --topic /topic/name --out /path/to/out.json
```

### Convert ROS 2 -> ROS 1

Single bag:

```bash
ros2utils convert_to_ros1 \
  --bag /path/to/bag.mcap \
  [--out /path/to/bag_ros1.bag] \
  [--include-topic /topic1 /topic2] \
  [--exclude-topic /topic3 /topic4] \
  [--remap /old:/new /old2:/new2] \
  [--src-typestore ros2_jazzy] \
  [--dst-typestore ros1_noetic] \
  [--split-duration 5m] \
  [--split-size 2G] \
  [--no-validate] \
  [--decode-ouster] \
  [--decode-ffmpeg] \
  [--output-mode points|depth|both] \
  [--preserve-lidar-fields] \
  [--lidar-topic /ouster/points/corrected]
```

Folder conversion:

```bash
ros2utils convert_folder \
  --folder /path/to/bag_folder \
  [--out-folder /path/to/output_folder] \
  [--ext .mcap .db3] \
  [--include-topic /topic1 /topic2] \
  [--exclude-topic /topic3 /topic4] \
  [--remap /old:/new] \
  [--no-skip] \
  [--src-typestore ros2_jazzy] \
  [--dst-typestore ros1_noetic] \
  [--split-duration 5m] \
  [--split-size 2G] \
  [--decode-ouster] \
  [--decode-ffmpeg] \
  [--output-mode points|depth|both] \
  [--points-topic /ouster/points] \
  [--depth-topic /ouster/depth_image] \
  [--imu-topic /ouster/imu] \
  [--keep-raw-ouster] \
  [--metadata-file /path/to/metadata.json] \
  [--no-validate]
```

Behavior notes:
- the default ROS 2 -> ROS 1 path in this repo is offline `rosbags` conversion, not live replay through `ros1_bridge`
- validation is enabled by default
- validation checks output timestamps and per-topic message counts
- `--split-duration` accepts plain seconds or values like `90s`, `5m`, `1h`
- `--split-size` accepts bytes or values like `500M`, `2G`
- chunk outputs are named `*_chunk_000.bag`, `*_chunk_001.bag`, and so on
- conversion is best-effort: the tool first tries the current `rosbags.convert()` API
  and then falls back to CLI (`rosbags-convert`) when needed
- message-typestore conversion is robust for standard ROS types, but custom/rare
  ROS2 types still depend on typestore compatibility and may be dropped
- `--decode-ouster` decodes Ouster packet topics into standard `sensor_msgs` topics
- `--decode-ffmpeg` decodes `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` topics into `sensor_msgs/Image`
- ffmpeg topics that end in `/ffmpeg` are decoded to the same topic without that suffix
- MCAP metadata/schema corruption can still appear as `KeyError` or `ros_distro`
  parse errors; in those cases only filtered known-good topics are typically preserved
- this repo defaults to `--src-typestore ros2_jazzy` and `--dst-typestore ros1_noetic`
- if conversion appears to stop mid-run, start with an explicit include set for
  the minimal topic set to avoid unsupported topic conversions

Docker note:
- when running through Docker, use `/bags/...` for `--bag`, `--out`, and `--folder`

Example in Docker:

```bash
BAGS_PATH=/path/to/bags_root docker compose -f Docker/ros/docker-compose.yml run --rm jazzy \
  ros2utils convert_to_ros1 \
    --bag /bags/session/run_01.mcap \
    --out /bags/session/run_01_ros1/ \
    --split-duration 10m
```

Decode Ouster + FFmpeg packet topics in Docker:

```bash
BAGS_PATH=/path/to/bags_root docker compose -f Docker/ros/docker-compose.yml run --rm jazzy \
  bash -lc 'source /opt/overlay_ws/install/setup.bash && \
  ros2utils convert_to_ros1 \
    --bag /bags/session/run_01.mcap \
    --out /bags/session/run_01_ros1/ \
    --decode-ouster \
    --decode-ffmpeg \
    --output-mode points'
```

## `ros1utils`

Global form:

```bash
ros1utils [--log-level {DEBUG,INFO,WARNING,ERROR}] [--log-file PATH] <command> ...
```

Common flags reused by many commands:

```bash
--user-config PATH --dev-config PATH --in INPUT --out OUTPUT --topics /topic1 /topic2 --report DIR
```

### Core bag editing

Calculate duration:

```bash
ros1utils calculate_bag_duration --in /path/to/bag.bag [--total]
```

Remove topics:

```bash
ros1utils remove_topic --in /path/to/bag.bag --out /path/to/output.bag --topics /topic_to_remove
```

Keep only selected topics:

```bash
ros1utils keep_topic --in /path/to/bag.bag --out /path/to/output.bag --topics /topic_to_keep1 /topic_to_keep2
```

Change `frame_id`:

```bash
ros1utils change_frame_id \
  --in /path/to/bag.bag \
  --out /path/to/output.bag \
  --topics /imu/data \
  --new-frame-id imu_link
```

Rename topics:

```bash
ros1utils remap_topics \
  --in /path/to/bag.bag \
  --remap /old/topic:/new/topic /old2:/new2 \
  [--out /path/to/output.bag] \
  [--overwrite]
```

If `--out` is omitted, the command writes a new `_remapped` bag by default.

Print topic sizes:

```bash
ros1utils print_topic_sizes --in /path/to/bag.bag
```

Convert IMU NED -> ENU:

```bash
ros1utils convert_imu_to_enu --in /path/to/bag.bag --out /path/to/output.bag --topics /imu/data
```

Rewrite `CameraInfo` messages:

```bash
ros1utils convert_camera_info --in /path/to/bag.bag --out /path/to/output.bag [--topics /camera/camera_info]
```

### Metrics and image workflows

Analyze dataset metrics:

```bash
ros1utils analyze_metrics \
  --in /path/to/bag.bag \
  --topics /cam/image_raw \
  --report /path/to/report_dir \
  [--benchmark]
```

Run illumination correction:

```bash
ros1utils auto_illumination \
  --in /path/to/bag.bag \
  --topics /cam/image_raw \
  --report /path/to/report_dir \
  [--save-bag /path/to/output.bag] [-f]
```

Colorize label images:

```bash
ros1utils colorize_labels \
  --in /path/to/bag.bag \
  --topics /labels/topic \
  [--out /path/to/output_dir] \
  [--color-map /path/to/colors.yaml] \
  [--interactive] [--save-all] [--save-both] \
  [--overlay-topic /camera/image_raw] \
  [--overlay-alpha 0.5] \
  [--overlay-max-dt 0.05] \
  [--stride 1] [--max-frames 0] [--resize 1.0]
```

Extract RGB images to PNG with timestamp manifest:

```bash
ros1utils extract_images_manifest \
  --in /path/to/bag.bag \
  --out /path/to/output_images \
  --topics /camera/color/image_raw \
  [--manifest-name image_manifest] \
  [--manifest-format {csv,txt,both}] \
  [--time-source {auto,header,bag}] \
  [--topic-discovery-timeout 5.0] \
  [--startup-timeout 25.0]
```

`--startup-timeout` enables immediate sequential partial scan (`read_messages()` on all topics)
and logs a warning if no selected image-topic message appears within that many seconds.
Extraction still continues and saves frames only from the selected image topic(s).

Rebuild a ROS1 image bag from manifest and PNG files:

```bash
ros1utils images_manifest_to_bag \
  --in /path/to/output_images/image_manifest.csv \
  --out /path/to/rebuilt.bag \
  [--images-root /path/to/output_images] \
  [--topics /camera/color/image_raw] \
  [--frame-id camera_color_optical_frame] \
  [--output-encoding {bgr8,rgb8,mono8}] \
  [--write-time {stamp,bag}] \
  [--manifest-delimiter {auto,comma,tab,semicolon}] \
  [--strict]
```

### NavSat tools

Export to CSV or KML:

```bash
ros1utils navsat_export \
  --in /path/to/bag.bag \
  --out /path/to/output_dir \
  --topics /fix \
  [--csv-name navsat.csv] [--kml-name navsat.kml]
```

Quick summary:

```bash
ros1utils navsat_summary --in /path/to/bag.bag --topics /fix
```

Detailed report:

```bash
ros1utils navsat_report --in /path/to/bag.bag --topics /fix --report /path/to/report_dir
```

### Metadata and calibration helpers

Extract metadata messages:

```bash
ros1utils extract_metadata \
  --in /path/to/bag.bag \
  [--out /path/to/metadata.jsonl] \
  [--topics /topic] \
  [--max-msgs 100]
```

Extract transform between URDF links:

```bash
ros1utils urdf_extrinsics \
  --urdf /path/to/robot.urdf \
  --from base_link \
  --to imu_link \
  [--rotation-only | --translation-only]
```

Wrapper-only shortcuts:

```bash
ros1utils crop_pointcloud_fov --in <in.bag> --out <out.bag> [--topic /ouster/points] [--fov-deg 120] [--center-deg 0]
```

These shortcuts call the packaged helper modules directly, so they do not depend on the current working directory.

## Sync Audit Status

Sync-audit utilities are not bundled in this checkout.
Use the split-out temporal-alignment repository noted in `docs/TEMPORAL_ALIGNMENT.md`.

## Standalone Scripts

Extract metadata from a ROS 1 bag topic:

```bash
ros1utils extract_metadata \
  --in /path/to/input.bag \
  --out /path/to/metadata.jsonl \
  [--topic /ouster/metadata]
```

Crop point cloud FOV:

```bash
ros1utils crop_pointcloud_fov \
  --in /path/to/bag.bag \
  --out /path/to/output_fov120.bag \
  [--topic /ouster/points] \
  [--fov-deg 120] \
  [--center-deg 0] \
  [--overwrite]
```

Convert by duration:

```bash
python3 scripts/convert_bags_per_duration.py \
  --input /path/to/bag.mcap \
  --output-folder /path/to/output_folder \
  [--duration 300] [--src-typestore ros2_jazzy] [--dst-typestore ros1_noetic]
```

Notes:
- accepts ROS 2 input bags in either `.mcap` or `.db3` format
- delegates splitting and conversion to `ros2utils convert_to_ros1`, so the wrapper and main CLI share one implementation

## Offline MAPIR NDVI

Derive NDVI from a ROS 1 bag and write a new bag containing only the derived NDVI topic(s):

```bash
ros1utils mapir_ndvi \
  --in /path/to/input.bag \
  --out /path/to/output_ndvi.bag \
  --image-topic /mapir/image_raw \
  --output-topic /mapir/indices/ndvi \
  --output-encoding 32FC1 \
  --publish-color \
  --color-topic /mapir/indices_color/ndvi \
  --colormap plant_health \
  --colorize-min -1.0 \
  --colorize-max 1.0 \
  --filter-set OCN
```

Run the same offline conversion through the FRUC ROS Docker stack:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose run --rm noetic \
  ros1utils mapir_ndvi \
    --in /bags/input.bag \
    --out /bags/output_ndvi.bag \
    --image-topic /mapir/image_raw \
    --output-topic /mapir/indices/ndvi \
    --publish-color \
    --color-topic /mapir/indices_color/ndvi \
    --colormap plant_health \
    --filter-set OCN
```

Useful overrides:

```bash
cd /home/forestsphere/work_utils/fruc_ros_utils/Docker/ros
BAGS_PATH=/path/to/bags_root docker compose run --rm noetic \
  ros1utils mapir_ndvi \
    --in /bags/input.bag \
    --out /bags/mapir_ndvi_only.bag \
    --image-topic /mapir/image_raw \
    --output-topic /mapir/indices/ndvi \
    --filter-set OCN \
    --publish-color \
    --color-topic /mapir/indices_color/ndvi \
    --colormap plant_health
```
