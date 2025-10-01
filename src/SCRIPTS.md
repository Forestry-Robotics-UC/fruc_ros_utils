# fruc_ros_utils – Scripts Overview

This document provides a clear description of each script in the repository, along with usage examples.

---

##  `src/bag`

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

### navsat_tools.py
**Purpose:**  
Utilities for GPS/Navigation Satellite data.  

**Functions:**
- `lla_to_ecef` — convert latitude/longitude/altitude → ECEF  
- `ecef_to_lla` — inverse conversion  
- `export_navsat_to_csv` — save NavSatFix-like dicts to CSV  
- `export_navsat_to_kml` — save trajectory as KML  

---

## `src/system`

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

## `src/vision`

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

# Notes
- **reports/** — contains auto-generated CSVs and figures from illumination and navsat processing  
- **utils/** — shared internal libraries for logging, metrics, TF, image processing  
