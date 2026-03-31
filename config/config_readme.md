# Configuration Guide (`fruc_ros_utils`)

This folder contains two configuration layers:

- `dev_defaults.yaml`: advanced defaults for maintainers and reproducible pipelines
- `user_config.yaml`: user-level overrides for day-to-day runs

Runtime precedence is:

`CLI args > user_config.yaml > dev_defaults.yaml`

## Supported Config Sections

The following top-level sections are merged by `bagutils`:

- `illumination`
- `bagutils`
- `mapir_ndvi`
- `colorize_labels`
- `extract_metadata`
- `navsat`
- `imu`
- `extrinsics`

## Common Workflows

### 1) Auto illumination

```bash
ros1utils auto_illumination \
  --user-config /path/to/config/user_config.yaml \
  --dev-config /path/to/config/dev_defaults.yaml \
  --in /bags/input.bag \
  --out /bags/output/
```

Main knobs: `illumination.*`, `bagutils.preserve_bayer`, `bagutils.exposure_time`.

### 2) MAPIR NDVI bag export

```bash
ros1utils mapir_ndvi \
  --user-config /path/to/config/user_config.yaml \
  --in /bags/input.bag \
  --out /bags/ndvi_output.bag
```

Main knobs: `mapir_ndvi.*` (`colormap`, `filter_set`, `publish_color`, channel overrides).

### 3) Label colorization

```bash
ros1utils colorize_labels \
  --user-config /path/to/config/user_config.yaml \
  --in /bags/input.bag \
  --topics /labels/topic
```

Main knobs: `colorize_labels.*` (`overlay_*`, `stride`, `max_frames`, `interactive`).

### 4) NavSat export/report

```bash
ros1utils navsat_export \
  --user-config /path/to/config/user_config.yaml \
  --in /bags/input.bag \
  --topics /gps/fix \
  --out /bags/reports
```

Main knobs: `navsat.csv_name`, `navsat.kml_name`.

## Notes

- Keep machine-specific paths only in `user_config.yaml`.
- Keep algorithmic defaults and tuning baselines in `dev_defaults.yaml`.
- CLI flags always win when debugging or A/B testing parameters.
