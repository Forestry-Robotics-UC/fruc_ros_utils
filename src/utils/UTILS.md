# fruc_ros_utils – Utils Overview

The `utils` package contains general-purpose utilities for logging, transforms, image processing, sensor conversions, and metrics.  
These modules are building blocks used across the repository.

---

## 📂 `utils/tf_utils.py`
**Purpose:**  
Helpers for handling TF and extrinsics (camera ↔ IMU calibration, URDF transforms).

**Main functions:**
- `build_R_cam_imu_per_topic(bag, topics, imu_frame, urdf_path=None, tf_static_topic="/tf_static")`  
  → Compute camera–IMU rotation matrices per topic from either URDF or `/tf_static`.  
- `load_extrinsics_yaml(path)`  
  → Load a 3×3 rotation matrix from YAML.  
- `load_extrinsics_from_urdf(urdf_path, link_a, link_b)`  
  → Compose transforms between URDF links.  

**Usage Example:**
```python
from utils.tf_utils import build_R_cam_imu_per_topic

R_map = build_R_cam_imu_per_topic(
    sample_bag_path="sample.bag",
    topics=["/camera/image_raw"],
    imu_frame="imu_link",
    urdf_path="robot.urdf"
)
```

---

## 📂 `utils/image_utils.py`
**Purpose:**  
Vision utilities for ROS ↔ OpenCV conversions, demosaicing, enhancement, and deblurring.

**Highlights:**
- **Bayer Handling**: `is_bayer`, `demosaic_bayer_ros`, `demosaic_bayer`, `bayer_to_code`, `remosaic_bgr`.  
- **Color Utilities**: `to_rgb`.  
- **Enhancements**: `gray_world_white_balance`, `gamma_correct`, `apply_clahe_on_l`, `retinex_msr`, `lime_enhance`.  
- **Deblurring**:  
  - `motion_kernel`, `richardson_lucy`, `wiener_deconv`  
  - `deblur_multiframe(center, context_frames, config, logger)`  
  - `deblur_single(img, imu_msg, K, exposure_time, config)`  

**Usage Example:**
```python
from utils import image_utils as vutils

img_bgr = vutils.gray_world_white_balance(img_bgr)
img_enhanced = vutils.apply_clahe_on_l(img_bgr, clip_limit=2.0, tiles=8)
```

---

## 📂 `utils/logging_utils.py`
**Purpose:**  
Lightweight logging utilities with colorized console output and optional file logging.

**Main components:**
- `ColorFormatter` — ANSI-colored log formatter.  
- `get_logger(name, level="INFO", log_file=None)`  
  → Returns a logger with colored console output and optional file handler.  

**Usage Example:**
```python
from utils.logging_utils import get_logger

logger = get_logger("Illumination", level="DEBUG", log_file="logs/run.log")
logger.info("This is a test message")
```

---

## 📂 `utils/sensor_conversions.py`
**Purpose:**  
Sensor frame convention conversions.

**Functions:**
- `imu_ned_to_enu(imu: sensor_msgs.msg.Imu) -> Imu`  
  → Convert IMU orientation, angular velocity, and acceleration from **NED** (North-East-Down) to **ENU** (East-North-Up).  

**Usage Example:**
```python
from utils.sensor_conversions import imu_ned_to_enu

imu_enu = imu_ned_to_enu(imu_msg_ned)
```

---

## 📂 `utils/vision.py`
**Purpose:**  
Vision-related **metrics** (sharpness, exposure, fusion weights).

**Functions:**
- `sharpness_score(img, method="tenengrad"|"laplacian"|"fft")`  
- `auto_threshold(scores, percentile=80)`  
- `exposure_metrics_robust(luma_u8)` → median/mean/std/dynamic range, %dark/%bright.  
- `sharpness_metrics(img_bgr)` → Laplacian variance & Tenengrad sharpness.  
- `sharpness_weight(gray, metric="laplacian", sigma=1.0)` → per-pixel weight maps.  

---

## 📂 `utils/navsat.py`
**Purpose:**  
GPS/Navigation Satellite covariance-based accuracy metrics.

**Functions:**
- `cov_metrics(cov: List[float]) -> Dict`  
  → Computes:  
  - σE, σN, σU (std. devs)  
  - σH (horizontal std. dev)  
  - r95 major/minor axes  
  - ellipse orientation (deg)  

**Usage Example:**
```python
from utils.metrics.navsat import cov_metrics

metrics = cov_metrics(msg.position_covariance)
print(metrics["r95_major"])
```

---

# Notes
- These utils are shared dependencies across `bag/`, `vision/`, and `system/` modules.  
- They are designed to be lightweight and ROS-friendly.  
