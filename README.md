# fruc_ros_utils

**fruc_ros_utils** is a collection of Python/ROS utilities for robotics perception and dataset processing.  
The tools are designed to be used **directly with Python 3** and ROS bag files, without requiring a `catkin_make` build.  

It provides helpers for:

- 📦 **ROS bag utilities** (filtering, extrinsics, IMU conversions, NavSat exports, topic metrics)  
- 📷 **Image processing** (illumination correction, sharp frame extraction, deblurring)  
- 🛰️ **Navigation satellite data** (CSV/KML exports, covariance-based metrics, quality reports)  
- 💻 **System monitoring** (CPU temps, core frequencies, topic rates, USB device monitoring)

---

## Repository Structure

```
fruc_ros_utils/
├── CMakeLists.txt        # Optional ROS integration (not required for basic use)
├── config/               # Default and user YAML configs
│   ├── config_readme.md  # Documentation for configuration
│   ├── dev_defaults.yaml
│   └── user_config.yaml
├── launch/               # Example ROS launch files
├── scripts/              # Standalone helper scripts
├── src/
│   ├── bag/              # Bag file utilities (bagutils, navsat tools)
│   ├── system/           # System monitoring and USB publishers
│   ├── vision/           # Vision modules (illumination, sharp image extraction, etc.)
│   └── utils/            # General-purpose utilities (logging, metrics, TF, conversions)
├── LICENSE
└── README.md
```

### Notes
- `reports/` — auto-generated outputs (illumination reports, GPS CSVs, figures).  
- `__pycache__/` — ignored Python bytecode.  

---

## Installation

Requires **Python 3.8+** and ROS 1 (for `rosbag`, `sensor_msgs`, etc).  

Install dependencies manually:

```bash
sudo apt-get install python3-rosbag python3-rospy python3-sensor-msgs
pip install numpy opencv-python tqdm pyyaml pyproj pandas matplotlib scikit-image
```

Or if a `requirements.txt` is provided:

```bash
pip install -r requirements.txt
```

---

## Usage Examples

Run scripts directly with `python3` (no `rosrun` required):

```bash
# Bag processing: illumination correction with reports
python3 src/bag/bagutils.py auto_illumination     --in input.bag --out out_dir     --topics /camera/image_raw --report reports/

# GPS export
python3 src/bag/bagutils.py navsat_export     --in input.bag --out exports --topics /gps/fix

# IMU conversion (NED → ENU)
python3 src/bag/bagutils.py convert_imu_to_enu     --in input.bag --out corrected.bag --topics /imu/data

# System monitoring (ROS node)
python3 src/system/system_monitoring.py
```

---

## Configuration

- Default settings: [`config/dev_defaults.yaml`](./config/dev_defaults.yaml)  
- User overrides: [`config/user_config.yaml`](./config/user_config.yaml)  
- See [config/config_readme.md](./config/config_readme.md) for details.  

---

## Documentation

- [SCRIPTS.md](./SCRIPTS.md) — high-level scripts (`bagutils`, `illumination`, `system_monitoring`, etc)  
- [UTILS.md](./UTILS.md) — reusable internal utilities (logging, TF, image helpers, metrics)  

---

## License

MIT License — free to use, modify, and distribute.
