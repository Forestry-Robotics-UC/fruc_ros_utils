# fruc_ros_utils — Modularization Refactoring Plan

Branch: `refactor/modularize-bag-utils`

## Motivation

Two god classes dominate the package:

| File | Lines | Class | Lines in class |
|---|---|---|---|
| `bag/ros2utils.py` | 3831 | `Ros2BagUtils` | ~3286 (lines 321–3607) |
| `bag/bagutils.py` | 1926 | `RosbagUtils` | ~1131 (lines 405–1537) |

Supporting problems:
- All subpackage `__init__.py` files except `bag/` are empty — no public API exposed.
- `bagutils.py` reads env vars at module level (lines 61–63) — import-time side effect.
- Four new converter scripts in `scripts/` duplicate `_ensure_rosbags()` bootstrapping.

**Goal:** each module answers one clear question. `Ros2BagUtils` and `RosbagUtils` become thin dispatchers.

## Rules (from memory)

- One responsibility per module. If the name needs "and", split again.
- Refactoring order: types → config → helpers → algorithms → pipeline → thin class.
- Each phase: extract → delegate → compile → test → commit. Do not clean internals during a move.
- Stop and ask if a method's behavior must change during extraction.
- No circular imports. No new global mutable state. No ROS imports in pure helpers.

---

## Phase 0 — Fix import-time side effects in `bagutils.py`

**Responsibility:** module-level env-var reads should not run at import time.

**What to change:**
- Lines 61–63: move `_default_level`, `_default_file`, and `logger` construction into `RosbagUtils.__init__` (or a `get_logger()` call guarded inside the class).
- Verify nothing else at module level triggers I/O or ROS imports.

**Verify:**
```bash
python3 -c "from fruc_ros_utils.bag.bagutils import RosbagUtils"
python3 -m py_compile src/fruc_ros_utils/bag/bagutils.py
```

**Commit:** `refactor(bagutils): remove import-time side effects`

---

## Phase 1 — Extract shared script bootstrap helper

**Responsibility:** `_ensure_rosbags()` is duplicated across four scripts; one canonical helper.

**Target:** `scripts/_deps.py` (private, not a package).

**What to move:**
- `_ensure_rosbags()` from `ros1_mcap_converter.py`, `convert_ros1_to_mcap.py`, `convert_ros1_chunks_to_mcap.py`, `simple_ros1_to_mcap.py`.
- Each script imports it: `from _deps import ensure_rosbags`.

**Verify:**
```bash
python3 -m py_compile scripts/_deps.py scripts/ros1_mcap_converter.py scripts/convert_ros1_to_mcap.py scripts/convert_ros1_chunks_to_mcap.py scripts/simple_ros1_to_mcap.py
```

**Commit:** `refactor(scripts): deduplicate _ensure_rosbags into _deps.py`

---

## Phase 2 — Extract `bagutils.py` config loading

**Responsibility:** load, merge, and validate YAML config trees. Has no bag I/O dependency.

**Target:** `bag/config.py`

**What to move from `bagutils.py`:**
- `make_cfg_tree` (line 306)
- `load_yaml` (line 353)
- `load_configs` (line 363)
- `merge_configs` (line 373)
- `section` helper (line 341)

**What stays:** `RosbagUtils` calls `load_configs` / `merge_configs` from `bag.config`.

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/config.py src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.config import load_configs, merge_configs"
```

**Commit:** `refactor(bagutils): extract config loading into bag/config.py`

---

## Phase 3 — Extract `bagutils.py` bag-editing primitives

**Responsibility:** read/write ROS1 bags without domain logic (no images, no navsat, no metrics).

**Target:** `bag/ros1_bag_ops.py`

**What to move from `bagutils.py`:**
- Private helpers: `_iter_bags`, `_iter_messages`, `_discover_bags`, `_is_image_datatype`, `_to_secs_nsecs`, `_sanitize_topic_for_path`, `_resolve_out_bag`, `_resolve_remap_out_bag`, `_alarm_handler`
- `RosbagUtils` methods: `calculate_bag_duration`, `remove_topic`, `remap_topics`, `print_topic_sizes`, `change_frame_id`, `convert_imu_to_enu`, `convert_camera_info`

**Extraction as functions:** these are stateless — extract as module-level functions, not a class.

**`RosbagUtils` delegates:**
```python
def calculate_bag_duration(self, in_path, total=False):
    return bag_ops.calculate_bag_duration(in_path, total, cfg=self._cfg)
```

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros1_bag_ops.py src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.ros1_bag_ops import calculate_bag_duration, remove_topic"
```

**Commit:** `refactor(bagutils): extract bag-editing primitives into ros1_bag_ops.py`

---

## Phase 4 — Extract `bagutils.py` image pipeline

**Responsibility:** extract, decode, manifest, and rebuild image streams from ROS1 bags.

**Target:** `bag/image_pipeline.py`

**What to move from `bagutils.py`:**
- Helpers: `_discover_image_topics`, `_run_with_timeout`, `_decode_image_to_bgr`, `_pick_stamp`, `_load_extrinsics`
- Methods: `extract_images`, `extract_images_manifest`, `images_manifest_to_bag`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/image_pipeline.py src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.image_pipeline import extract_images_manifest, images_manifest_to_bag"
```

**Commit:** `refactor(bagutils): extract image pipeline into image_pipeline.py`

---

## Phase 5 — Extract `bagutils.py` navsat pipeline

**Responsibility:** extract and export GPS/NavSat data from ROS1 bags.

**Target:** merge into existing `bag/navsat_tools.py` (already has `export_navsat_to_csv`, `export_navsat_to_kml`).

**What to move from `bagutils.py`:**
- `extract_navsat_records`, `navsat_export`, `navsat_summary`, `navsat_report`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/navsat_tools.py src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.navsat_tools import navsat_export, navsat_summary"
```

**Commit:** `refactor(bagutils): move navsat methods into navsat_tools.py`

---

## Phase 6 — Extract `bagutils.py` metrics and illumination pipeline

**Responsibility:** compute exposure/sharpness metrics and run illumination correction over bags.

**Target:** `bag/metrics_pipeline.py`

**What to move from `bagutils.py`:**
- `analyze_metrics`, `auto_illumination_from_bag`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/metrics_pipeline.py src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.metrics_pipeline import analyze_metrics, auto_illumination_from_bag"
```

**Commit:** `refactor(bagutils): extract metrics/illumination pipeline`

---

## Phase 7 — Extract `bagutils.py` NDVI pipeline

**Responsibility:** derive NDVI from a ROS1 bag and write output bag.

**Target:** `bag/ndvi_pipeline.py`

**What to move from `bagutils.py`:**
- `mapir_ndvi` and any helpers it uses exclusively.

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ndvi_pipeline.py src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.ndvi_pipeline import mapir_ndvi"
```

**Commit:** `refactor(bagutils): extract NDVI pipeline into ndvi_pipeline.py`

---

## Phase 8 — `RosbagUtils` becomes a thin dispatcher

At this point `RosbagUtils` should only:
- load config once in `__init__`
- delegate every public method to the extracted modules

Target size: under 100 lines.

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/bagutils.py
python3 -c "from fruc_ros_utils.bag.bagutils import RosbagUtils; r = RosbagUtils()"
pytest tests/ -x -q  # if tests exist
wc -l src/fruc_ros_utils/bag/bagutils.py  # should be < 150
```

**Commit:** `refactor(bagutils): RosbagUtils is now a thin dispatcher`

---

## Phase 9 — Extract `ros2utils.py` module-level private helpers

**Responsibility:** pure utility functions with no class state (parsing, formatting, logging helpers).

**Target:** `bag/_ros2_helpers.py` (private module, not exported)

**What to move (all module-level functions before line 321):**
- `_make_progress_bar`, `_debug_path_state`, `_debug_topic_count_summary`
- `_is_standard_ros_msg_type`, `_is_ffmpeg_packet_msg_type`
- `_safe_rosbag2_metadata_scalar`, `_merge_topic_lists`
- `_parse_threshold_value`, `_parse_size_bytes`, `_parse_duration_seconds`
- `_format_bytes`, `_natural_sort_key`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/_ros2_helpers.py src/fruc_ros_utils/bag/ros2utils.py
```

**Commit:** `refactor(ros2utils): extract module-level helpers into _ros2_helpers.py`

---

## Phase 10 — Extract `ros2utils.py` bag I/O layer

**Responsibility:** open, read metadata, and clean up ROS2 bags. No conversion logic.

**Target:** `bag/ros2_bag_io.py`

**What to move from `Ros2BagUtils`:**
- `_open_reader`, `_storage_id_for_bag`, `_bag_size_bytes_raw`
- `_ros2_bag_info_metadata`, `_single_file_rosbag2_metadata`
- `_prepare_rosbags_convert_input`, `_rewrite_ros2_bag_for_rosbags`
- `_ensure_output_available`, `_cleanup_partial_output`
- `_ensure_ros1_tf2_typestore`, `_resolve_rosbags_typestore`
- `_debug_log_opened_ros2_reader`, `_debug_log_ros2_bag_summary`
- `_debug_log_topic_descriptions`, `_debug_log_subprocess_result`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2_bag_io.py src/fruc_ros_utils/bag/ros2utils.py
```

**Commit:** `refactor(ros2utils): extract bag I/O layer into ros2_bag_io.py`

---

## Phase 11 — Extract `ros2utils.py` inspection

**Responsibility:** read-only queries over ROS2 bags (topics, duration, JSON).

**Target:** `bag/ros2_inspector.py`

**What to move from `Ros2BagUtils`:**
- `list_topics`, `info`, `bag_duration`, `bag_size_bytes`
- `extract_json`, `_first_json_string_topic`, `_try_extract_json`, `_get_msg_class`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2_inspector.py src/fruc_ros_utils/bag/ros2utils.py
python3 -c "from fruc_ros_utils.bag.ros2_inspector import Ros2Inspector"
```

**Commit:** `refactor(ros2utils): extract inspection into ros2_inspector.py`

---

## Phase 12 — Extract `ros2utils.py` topic selection and filtering

**Responsibility:** decide which topics to include/exclude/remap for conversion.

**Target:** `bag/ros2_topic_filter.py`

**What to move from `Ros2BagUtils`:**
- `_topic_selected`, `_topics_for_msg_type`, `_selected_topic_names`
- `_auto_exclude_unsupported_topics`, `_warn_custom_types`
- `_source_topic_types`, `_expected_topic_counts`, `_remap_topics`
- `_extract_parse_failed_msgtype`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2_topic_filter.py src/fruc_ros_utils/bag/ros2utils.py
```

**Commit:** `refactor(ros2utils): extract topic filtering into ros2_topic_filter.py`

---

## Phase 13 — Extract `ros2utils.py` validation

**Responsibility:** verify output bag integrity (timestamps, message counts).

**Target:** `bag/ros2_validation.py`

**What to move from `Ros2BagUtils`:**
- `_validate_timestamps`, `_validate_message_counts`
- `_count_ros2_messages_per_topic`, `_count_ros1_messages_per_topic`
- `_validate_ouster_packet_topic`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2_validation.py src/fruc_ros_utils/bag/ros2utils.py
python3 -c "from fruc_ros_utils.bag.ros2_validation import validate_message_counts"
```

**Commit:** `refactor(ros2utils): extract validation into ros2_validation.py`

---

## Phase 14 — Extract `ros2utils.py` Ouster helpers

**Responsibility:** Ouster-specific conversion: parse metadata, decode packets, validate lidar topics.

**Target:** `bag/ouster_conv_helpers.py`

**What to move from `Ros2BagUtils`:**
- `_coerce_int`, `_parse_ouster_lidar_mode`, `_parse_ouster_channel_count`
- `_ouster_packets_per_scan`, `_pick_ouster_points_topic`
- `_restore_lidar_fields_from_mcap`
- `_convert_single_to_ros1_with_ouster_decode`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ouster_conv_helpers.py src/fruc_ros_utils/bag/ros2utils.py
```

**Commit:** `refactor(ros2utils): extract Ouster conversion helpers`

---

## Phase 15 — Extract `ros2utils.py` ffmpeg helpers

**Responsibility:** discover and map ffmpeg-encoded topics for decoding during conversion.

**Target:** `bag/ffmpeg_conv_helpers.py`

**What to move from `Ros2BagUtils`:**
- `_ffmpeg_decoded_output_topic`, `_discover_ffmpeg_decode_topic_map`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ffmpeg_conv_helpers.py src/fruc_ros_utils/bag/ros2utils.py
```

**Commit:** `refactor(ros2utils): extract ffmpeg helpers`

---

## Phase 16 — Extract `ros2utils.py` split management

**Responsibility:** decide when and how to split a ROS2 bag before conversion.

**Target:** `bag/ros2_splitter.py`

**What to move from `Ros2BagUtils`:**
- `_resolve_split_output_base`, `_should_split_for_conversion`
- `_discover_split_chunk_sources`, `_existing_split_member_sources`, `_existing_split_member_count`
- `_split_ros2_bag`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2_splitter.py src/fruc_ros_utils/bag/ros2utils.py
```

**Commit:** `refactor(ros2utils): extract split management into ros2_splitter.py`

---

## Phase 17 — Extract `ros2utils.py` conversion core

**Responsibility:** orchestrate single-bag and multi-path ROS2→ROS1 conversion.

**Target:** `bag/ros2_converter.py`

**What to move from `Ros2BagUtils`:**
- `convert_to_ros1`, `_convert_single_to_ros1`
- `_convert_to_ros1_with_split`, `_convert_existing_split_members_to_ros1`
- `_convert_using_rosbags_api`, `_convert_using_rosbag2_py_writer_fallback`
- `_convert_using_rosbags_cli_fallback`, `_convert_using_rosbags_cli_or_rewrite_fallback`
- `_convert_using_rosbags_convert_fallback`

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2_converter.py src/fruc_ros_utils/bag/ros2utils.py
python3 -c "from fruc_ros_utils.bag.ros2_converter import Ros2Converter"
```

**Commit:** `refactor(ros2utils): extract conversion core into ros2_converter.py`

---

## Phase 18 — `Ros2BagUtils` becomes a thin dispatcher

At this point `Ros2BagUtils` should only:
- instantiate the extracted components
- expose the same public methods as a stable facade
- delegate every call

Target size: under 150 lines.

**Verify:**
```bash
python3 -m py_compile src/fruc_ros_utils/bag/ros2utils.py
python3 -c "from fruc_ros_utils.bag.ros2utils import Ros2BagUtils; b = Ros2BagUtils()"
wc -l src/fruc_ros_utils/bag/ros2utils.py  # should be < 200
```

**Commit:** `refactor(ros2utils): Ros2BagUtils is now a thin dispatcher`

---

## Phase 19 — Wire up public `__init__.py` exports

**Responsibility:** define the public API for each subpackage so callers don't import from deep internals.

**Targets:**

`bag/__init__.py` — already has `OusterPointCloudEncoder`. Add:
```python
from .ros1_bag_ops import calculate_bag_duration, remove_topic, remap_topics
from .ros2_inspector import Ros2Inspector
from .ros2_converter import Ros2Converter
from .image_pipeline import extract_images_manifest, images_manifest_to_bag
from .navsat_tools import navsat_export, navsat_summary
from .ouster_encode import OusterPointCloudEncoder
from .ouster_decode import OusterPacketDecoder
```

`utils/__init__.py`, `vision/__init__.py`, `system/__init__.py` — add at minimum the most-used public symbols.

**Verify:**
```bash
python3 -c "from fruc_ros_utils.bag import Ros2Inspector, Ros2Converter, calculate_bag_duration"
python3 -c "from fruc_ros_utils.utils import get_logger"
python3 -c "from fruc_ros_utils.vision import IlluminationEnhancer"
```

**Commit:** `refactor: wire up public __init__.py exports across subpackages`

---

## Definition of done

A phase is complete when:
- `py_compile` passes on all changed files
- Any existing tests still pass (`pytest tests/ -x -q`)
- `Ros2BagUtils` and `RosbagUtils` still expose the same public methods (backward compat)
- No circular imports introduced
- No behavior changed — only responsibility boundaries shifted

Full refactor is done when:
- `bagutils.py` class body < 150 lines
- `ros2utils.py` class body < 200 lines
- Each extracted module can be described in one sentence
- `from fruc_ros_utils.bag import X` works for all major entry points
