# fruc_ros_utils – Scripts Overview

This document provides a clear description of each script in the repository, along with usage examples.

---

##  `src/fruc_ros_utils/bag`

### ouster_encode.py

**Purpose:**
Re-encodes decoded Ouster data back into raw packet topics (inverse of `ouster_decode.py`).
Converts `sensor_msgs/PointCloud2` → `ouster_sensor_msgs/PacketMsg` (lidar packets) + `std_msgs/String` (metadata).

**Main class: `OusterPointCloudEncoder`**
- `from_metadata_string(meta_str)` — classmethod, initialize from sensor metadata JSON string
- `process_message(topic, data, ts)` → list of `(topic, type, msg, ts)` output tuples
- `output_topic_types()` — dict of output topic → message type strings
- `raw_topics()` — list of input topics this encoder consumes
- `get_stats()` — dict of processing counters

**Usage:**
```python
from fruc_ros_utils.bag.ouster_encode import OusterPointCloudEncoder

encoder = OusterPointCloudEncoder.from_metadata_string(meta_json_str)
for out_topic, out_type, out_msg, out_ts in encoder.process_message(topic, data, ts):
    writer.write(out_topic, rclpy.serialization.serialize_message(out_msg), out_ts)
```

---

### ffmpeg_encode.py

**Purpose:**
Encodes `sensor_msgs/Image` topics into `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` topics
(inverse of `ffmpeg_decode.py`). Uses PyAV (`av` module) for libx264 encoding.

**Note:** Returns pre-serialized CDR bytes (not rclpy messages) because `ffmpeg_image_transport_msgs`
has no rclpy Python bindings in the fruc-jazzy overlay.  Write the bytes to the bag directly.

**Main class: `FFMPEGPacketEncoder`**
- `process_message(topic, data, ts)` → list of `(topic, type, cdr_bytes, ts)` — `cdr_bytes` is `bytes`
- `flush(topic)` → flush remaining B-frames
- `output_topic_types()` — dict of output topic → message type strings
- `raw_topics()` — list of input topics this encoder consumes
- `get_stats()` — dict of processing counters

**Usage:**
```python
from fruc_ros_utils.bag.ffmpeg_encode import FFMPEGPacketEncoder

encoder = FFMPEGPacketEncoder(topic_map={"/mapir/image_raw": "/mapir/image_raw/ffmpeg"})
for out_topic, out_type, cdr_bytes, out_ts in encoder.process_message(topic, data, ts):
    writer.write(out_topic, cdr_bytes, out_ts)  # cdr_bytes already serialized
```

---

### bagutils.py
**Purpose:**  
General-purpose utilities for processing ROS bag files.  
Supports topic removal, frame ID changes, IMU conversion, NavSat exports, illumination correction, and more.  

**Main functions:**
- `calculate_bag_duration` — compute duration of one or more bags  
- `remove_topic` — remove specific topics  
- `print_topic_sizes` — summarize total message sizes by topic  
- `change_frame_id` — update frame IDs for messages  
- `convert_imu_to_enu` — convert IMU orientation from NED → ENU  
- `extract_navsat_records` — extract GPS-like records  
- `navsat_export` — export NavSatFix to CSV/KML  
- `navsat_summary` — summary stats of GPS quality  
- `navsat_report` — generate CSV/JSON reports (requires pandas)  
- `extract_images` — extract raw images from bag topics  
- `analyze_metrics` — compute exposure/sharpness metrics  
- `auto_illumination_from_bag` — batch illumination correction with optional egomotion compensation  

**CLI usage:**
```bash
python bagutils.py calculate_bag_duration --in mybag.bag --total
python bagutils.py remove_topic --in input.bag --out output.bag --topics /camera/image_raw
python bagutils.py convert_imu_to_enu --in input.bag --out corrected.bag --topics /imu/data
python bagutils.py navsat_export --in input.bag --out exports --topics /gps/fix --csv-name gps.csv
python bagutils.py auto_illumination --in input.bag --out outdir --topics /camera/image_raw --report reports/
```

---

### ros2utils.py

**Purpose:**
Entry point and thin dispatcher for all ROS 2 bag operations. `Ros2BagUtils` is a stateless class whose methods delegate to the specialized modules below. The module also provides a `ros2utils` CLI entry point.

**Main class: `Ros2BagUtils`**
All methods delegate to module-level functions — see individual modules for logic.

Key public methods (same signatures as before the modular refactor):
- `list_topics(bag_path)` — dict of topic → message type
- `bag_duration(bag_path)` — duration in seconds
- `bag_size_bytes(bag_path)` — total size in bytes
- `info(bag_path)` — dict with duration, size, topic count, topics
- `convert_to_ros1(bag_path, ...)` — convert a single ROS 2 bag to ROS 1 `.bag`
- `convert_folder_to_ros1(folder, ...)` — batch convert a folder of bags

**CLI usage:**
```bash
ros2utils info --bag session.mcap
ros2utils convert_to_ros1 --bag session.mcap --out session_ros1.bag
```

---

### ros2\_inspector.py

**Purpose:**
ROS 2 bag I/O and inspection. Opens bags, queries metadata, reports duration and size. No conversion or topic filtering logic.

**Public functions:**
- `open_ros2_reader(path)` — open a `rosbag2_py.SequentialReader`
- `list_topics(bag_path)` — `{topic: msg_type}` dict
- `bag_size_bytes(bag_path)` — total on-disk size in bytes
- `bag_size_bytes_raw(bag_path)` — raw size without metadata overhead
- `bag_duration(bag_path)` — recording duration in seconds
- `ensure_output_available(dst_path, overwrite)` — guard before writing
- `info(bag_path)` — summary dict (duration, size, topics)

---

### ros2\_topic\_filter.py

**Purpose:**
Topic selection and filtering for ROS 2 bags. Decides which topics to include, exclude, or remap. Also discovers ffmpeg packet topics.

**Public functions:**
- `_topic_selected(topic_name, include_topics, exclude_topics)` — boolean filter
- `_source_topic_types(bag_path)` — `{topic: msg_type}` from reader
- `_selected_topic_names(bag_path, include_topics, exclude_topics)` — filtered topic list
- `_topics_for_msg_type(bag_path, msg_type, ...)` — topics matching a type
- `_auto_exclude_unsupported_topics(bag_path, ...)` — drop non-standard ROS types
- `_discover_ffmpeg_decode_topic_map(topic_types, ...)` — build ffmpeg input→output map

---

### ros2\_validation.py

**Purpose:**
Post-conversion validation. Checks per-topic message counts and header timestamps. Imports from `ros2_inspector` and `ouster_conv_helpers` for Ouster packet-to-scan verification.

**Public functions:**
- `_warn_custom_types(bag_path)` — warn if bag contains non-standard message types
- `_validate_timestamps(bag_path, max_drift_s)` — check header vs. bag timestamp drift
- `_count_ros2_messages_per_topic(bag_path)` — count source messages by topic
- `_count_ros1_messages_per_topic(bag_path)` — count output messages by topic
- `_expected_topic_counts(src_counts, include_topics, exclude_topics, remap)` — compute expected counts after filtering
- `_validate_ouster_packet_topic(src_path, packet_topic, packet_count, dst_counts)` — validate lidar packet → point cloud scan count ratio
- `_validate_message_counts(src_path, dst_path, ...)` — full message-count validation pass

---

### ouster\_conv\_helpers.py

**Purpose:**
Ouster LiDAR specific helpers: sensor metadata parsing, packets-per-scan inference, point topic selection, and lidar field restoration from MCAP.

**Public functions:**
- `_get_msg_class(type_str)` — dynamically import a ROS message class by type string
- `_first_json_string_topic(bag_path, topic_name)` — deserialize first message from a JSON string topic
- `_coerce_int(value)` — safe int cast
- `_parse_ouster_lidar_mode(lidar_mode)` — extract columns/frame and scan rate from mode string
- `_parse_ouster_channel_count(model)` — extract channel count from model string
- `_ouster_packets_per_scan(bag_path, prefix)` — read metadata topic and infer packets/scan
- `_pick_ouster_points_topic(dst_counts, prefix)` — select best point cloud output topic
- `_restore_lidar_fields_from_mcap(mcap_path, bag_path, topic)` — inject ring/reflectivity fields from MCAP into ROS 1 bag

---

### ros2\_converter.py

**Purpose:**
Full ROS 2 → ROS 1 conversion pipeline and split management. Contains no bag-opening or topic-filtering logic; delegates to the inspector, filter, validation, and ouster modules.

**Public functions:**
- `convert_single_to_ros1(bag_path, out_path, ...)` — convert one bag; auto-selects rosbags API → CLI → fallback chain
- `convert_single_to_ros1_with_ouster_decode(bag_path, out_path, ...)` — direct decode path with Ouster and ffmpeg decoders
- `convert_to_ros1_with_split(bag_path, ...)` — split-then-convert pipeline
- `convert_existing_split_members_to_ros1(bag_path, ...)` — convert pre-split member files

Internal fallback chain (in order):
1. `_convert_using_rosbags_api` — rosbags Python API (Ros2Reader → Ros1Writer)
2. `_convert_using_rosbags_convert_fallback` — `rosbags.convert` Python call
3. `_convert_using_rosbags_cli_or_rewrite_fallback` — CLI + sqlite3 rewrite loop
4. `_convert_using_rosbag2_py_writer_fallback` — direct rosbag2\_py reader → rosbags Ros1Writer

---

### navsat_tools.py
**Purpose:**  
Utilities for GPS/Navigation Satellite data.  

**Functions:**
- `lla_to_ecef` — convert latitude/longitude/altitude → ECEF  
- `ecef_to_lla` — inverse conversion  
- `export_navsat_to_csv` — save NavSatFix-like dicts to CSV  
- `export_navsat_to_kml` — save trajectory as KML  

---

## `src/fruc_ros_utils/system`

### system_monitoring.py
**Purpose:**  
Publishes system metrics to `/diagnostics/system`.  

**Monitored values:**
- CPU temperatures (via `/sys/class/thermal/...`)  
- CPU frequencies (current & max per core)  
- ROS topic frequencies (via `rostopic hz`)  

**ROS Usage:**
```bash
rosrun fruc_ros_utils system_monitoring.py
```

Publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics/system`.

---

### usb_buffer_publisher.py
**Purpose:**  
Monitor USB devices and capture traffic.  

**Functions:**
- `get_usb_devices()` — list connected USB devices  
- `capture_usb_traffic()` — measure traffic using `usbmon`  
- `get_device_descriptor()` — extract device descriptor info  

**Standalone usage:**
```bash
python usb_buffer_publisher.py
```
Prints device classes and estimated traffic.

---

## `src/fruc_ros_utils/vision`

### illumination.py
**Purpose:**  
Enhance image illumination with:  
- White balance  
- CLAHE contrast stretching  
- Gamma correction  
- Optional egomotion correction (using IMU + intrinsics)  
- Optional deblurring (single or multiframe)  

**Main Classes:**
- `IlluminationConfig` — tunable thresholds  
- `IlluminationEnhancer` — corrects single frames or whole bags  

**Usage via bagutils CLI:**
```bash
python bagutils.py auto_illumination --in input.bag --out outdir --topics /camera/image_raw --report reports/
```

---

### save_sharp_images.py
**Purpose:**  
Extracts sharp images from ROS bags. Uses sharpness metrics to filter blurred frames.  

**Features:**
- Supports metrics: Tenengrad, Laplacian, FFT  
- Option to auto-threshold by percentile  
- Balanced sampling across multiple bags  

**CLI usage:**
```bash
python save_sharp_images.py     --in my_bags/     --out sharp_images/     --topic /camera/image_raw     --max-images 200     --method tenengrad     --auto-percentile 85
```

---

## `scripts/`

### ros1_mcap_converter.py

**Purpose:**
Convert ROS1 `.bag` chunks to MCAP using the `rosbags` Python API directly (low-level, no subprocess).
Iterates a set of chunk-indexed bag files and writes one `.mcap` per chunk.

**Key function:** `convert_ros1_chunk_to_mcap(bag_path, output_path) -> bool`

**CLI usage:**
```bash
python3 scripts/ros1_mcap_converter.py \
  --bags-dir /bags \
  --output-dir /bags_out \
  --name-prefix 2026_03_27_17_02_24__icnf-curt \
  --chunk-indices 5 6 7 8 9 \
  [--verbose]
```

---

### convert_ros1_to_mcap.py

**Purpose:**
Convert a single ROS1 bag to MCAP using the `rosbags-convert` CLI (subprocess).
Handles intermediate rosbag2 directory and repacks to a flat `.mcap` file.

**Key function:** `convert_ros1_to_mcap(bag_path, output_path) -> bool`

**CLI usage:**
```bash
python3 scripts/convert_ros1_to_mcap.py \
  --bags-dir /bags \
  --output-dir /bags_out \
  --name-prefix 2026_03_27_17_02_24__icnf-curt \
  --chunk-indices 5 6 7 8 9 \
  [--compress] [--verbose]
```

---

### convert_ros1_chunks_to_mcap.py

**Purpose:**
Batch-convert a set of ROS1 chunk bags to MCAP, filling missing sequence numbers.
Each chunk is converted independently; output files are named by chunk index.

**CLI usage:**
```bash
python3 scripts/convert_ros1_chunks_to_mcap.py \
  --bags-dir /bags \
  --output-dir /bags_out \
  --name-prefix 2026_03_27_17_02_24__icnf-curt \
  --chunk-indices 5 6 7 8 9 \
  [--compress] [--overwrite] [--verbose]
```

---

### simple_ros1_to_mcap.py

**Purpose:**
Minimal ROS1 → MCAP converter using `rosbags-convert` via subprocess.
Converts via a temporary rosbag2 directory and repacks to a flat `.mcap`.
Preferred when a lightweight, dependency-minimal path is needed.

**CLI usage:**
```bash
python3 scripts/simple_ros1_to_mcap.py \
  --bags-dir /bags \
  --output-dir /bags_out \
  --name-prefix 2026_03_27_17_02_24__icnf-curt \
  --chunk-indices 5 6 7 8 9 \
  [--verbose]
```

---

# Notes
- **reports/** — contains auto-generated CSVs and figures from illumination and navsat processing  
- **utils/** — shared internal libraries for logging, metrics, TF, image processing  
