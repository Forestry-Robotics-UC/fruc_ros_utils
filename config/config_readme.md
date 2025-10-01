# Configuration Guide — `fruc_ros_utils`

This document describes all configuration parameters available in the YAML configs:

- `user_config.yaml` — simple, safe options for end users
- `dev_config.yaml` — full set of developer/advanced options
- Defaults are embedded in the code

At runtime: **user_config > dev_config > defaults**.  
This allows safe user overrides while keeping developer flexibility.

---

## 📑 Table of Contents
1. [User Parameters](#-user-parameters)
2. [Developer Parameters](#-developer-parameters)
   - [Illumination](#illumination)
   - [Deblurring & Motion](#deblurring--motion)
   - [Image Utilities](#image-utilities)
   - [Metrics & Sharpness](#metrics--sharpness)
   - [IMU / Extrinsics](#imu--extrinsics)
   - [Bagutils / I/O](#bagutils--io)

---

## 🟢 User Parameters

These are safe, high-level knobs that most users will interact with.

| Parameter | Type / Options | Default | Description |
|-----------|----------------|---------|-------------|
| `white_balance` | bool | `true` | Apply gray-world white balance. |
| `under_method` | `dynamic`, `clahe`, `gamma`, `msr`, `lime` | `dynamic` | Method for underexposed correction. |
| `over_method` | `dynamic`, `reinhard`, `drago`, `mantiuk`, `clahe`, `msr`, `lime` | `dynamic` | Method for overexposed correction. |
| `deblur_enabled` | bool | `true` | Toggle motion deblurring. |
| `deblur_mode` | `single`, `multiframe`, `off` | `multiframe` | Select deblurring type. |
| `mean_dark` | float | 85.0 | Threshold below which image is underexposed. |
| `mean_bright` | float | 170.0 | Threshold above which image is overexposed. |
| `force` | bool | `false` | Force correction even if image looks “good.” |
| `report` | path | `"reports/"` | Directory to save reports. |
| `in_path` | path | `"bags/"` | Input data path. |
| `out_path` | path | `"bags_corrected/"` | Output path. |
| `save_bag` | string/null | null | Name of corrected ROS bag (if saving). |
| `topics` | list | `["/camera/image_raw"]` | Topics to process. |
| `imu_frame` | string | `"imu_link"` | IMU frame id. |
| `urdf_path` | path/null | null | Path to URDF for extrinsics. |
| `tf_static_topic` | string | `"/tf_static"` | TF static topic. |

---

## 🔵 Developer Parameters

These options control algorithm internals and advanced tuning.  
Grouped by subsystem with references to theory.

---

### Illumination

| Parameter | Default | Description | Theory / References |
|-----------|---------|-------------|----------------------|
| `clahe_clip_limit` | 2.8 | CLAHE contrast limit | [OpenCV CLAHE Docs](https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html?utm_source=chatgpt.com) |
| `clahe_tiles` | 8 | CLAHE tile grid size | — |
| `target_mean_under` / `target_mean_over` | 135.0 / 120.0 | Target mean luminance for gamma | — |
| `pct_dark_th` / `pct_bright_th` | 25.0 / 20.0 | % pixels clipped to flag under/overexposure | — |
| `dyn_range_min` | 60.0 | Min dynamic range (p99 – p1) before “low contrast” | — |
| `gamma_min` / `gamma_max` | 0.6 / 1.8 | Clamps for gamma correction | — |
| `raw_enable_clahe` | false | Apply CLAHE to raw Bayer before demosaic | — |
| `raw_p_low` / `raw_p_high` | 1.0 / 99.0 | Percentile clipping for raw correction | — |
| `raw_gamma_strength` | 0.7 | Gamma strength exponent for raw | — |
| `ego_theta_thresh_rad` | 0.0025 | Egomotion threshold (rad) to apply rotation correction | — |

---

### Deblurring & Motion

| Parameter | Default | Description | Theory / References |
|-----------|---------|-------------|----------------------|
| `deblur_algorithm` | `"lucy"` | `"lucy"` (Richardson–Lucy) or `"wiener"` | Richardson–Lucy: iterative Poisson deconvolution ([Prato et al., arXiv:1210.2258](https://arxiv.org/abs/1210.2258?utm_source=chatgpt.com)) |
| `deblur_iters` | 15 | RL iteration count | Convergence & noise trade-off ([NIH PMC article](https://pmc.ncbi.nlm.nih.gov/articles/PMC11751374/?utm_source=chatgpt.com)) |
| `deblur_wiener_k` | 0.01 | Wiener noise constant | Wiener filter = closed form deconvolution ([Stanford CS231m notes](http://cs231m.stanford.edu/lectures/2021/lecture13.pdf?utm_source=chatgpt.com)) |
| `deblur_psf_min_px` / `deblur_psf_max_px` | 2.0 / 25.0 | Min/max blur kernel length | — |
| `mf_window` | 3 | Multiframe fusion window (odd: 3, 5, …) | Multi-frame fusion in computational photography |
| `mf_align` | `"flow"` | `"flow"` (optical flow) or `"ego"` (IMU) | Optical flow alignment ([Horn & Schunck, 1981](https://doi.org/10.1016/0004-3702(81)90024-2?utm_source=chatgpt.com)) |
| `mf_sharpness_metric` | `"laplacian"` | Sharpness measure (Laplacian/Sobel) | Laplacian variance as sharpness estimator |
| `mf_sigma` | 1.0 | Gaussian smoothing sigma for fusion weights | — |

---

### Image Utilities

| Parameter | Default | Description | Theory / References |
|-----------|---------|-------------|----------------------|
| `retinex_scales` | `[15, 80, 250]` | Scales for Multi-Scale Retinex (MSR) | *Retinex Processing for Automatic Image Enhancement* (Majumder) [PDF](https://ics.uci.edu/~majumder/vispercep/retinexenhancement.pdf?utm_source=chatgpt.com) |
| `lime_guided_radius` | 15 | Guided filter radius in LIME | Original LIME paper ([Guo et al., TIP 2017](https://ieeexplore.ieee.org/document/7782813?utm_source=chatgpt.com)) |
| `lime_guided_eps` | 0.001 | Guided filter regularization | see LIME reference above |
| `motion_kernel_width` | 0.8 | Gaussian width for synthetic PSF | — |

---

### Metrics & Sharpness

| Parameter | Default | Description | Theory / References |
|-----------|---------|-------------|----------------------|
| `sharpness_metric` | `"laplacian"` | Sharpness estimator (`laplacian` or `sobel`) | Variance of Laplacian ([Pertuz et al., "Analysis of focus measure operators"](https://www.sciencedirect.com/science/article/pii/S003132031200278X?utm_source=chatgpt.com)) |
| `sobel_ksize` | 3 | Sobel kernel size | — |
| `laplacian_ksize` | 3 | Laplacian kernel size | — |

---

### IMU / Extrinsics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `imu_frame` | `"imu_link"` | IMU frame identifier |
| `tf_static_topic` | `"/tf_static"` | TF static topic name |
| `urdf_path` | null | URDF file for extrinsics |
| `extrinsics_yaml` | null | YAML calibration file |

---

### Bagutils / I/O

| Parameter | Default | Description |
|-----------|---------|-------------|
| `parallel` | false | Enable multiprocessing |
| `preserve_bayer` | true | Save Bayer format instead of demosaic BGR |
| `exposure_time` | 0.01 | Exposure time (s) for blur estimation |
| `force` | false | Force corrections on all images |

---

