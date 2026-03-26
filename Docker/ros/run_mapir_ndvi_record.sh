#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

INPUT_BAG="${MAPIR_INPUT_BAG:-/bags/input.bag}"
OUTPUT_BAG="${MAPIR_OUTPUT_BAG:-/bags/mapir_ndvi.bag}"
IMAGE_TOPIC="${MAPIR_IMAGE_TOPIC:-/mapir/image_raw}"
OUTPUT_TOPIC="${MAPIR_NDVI_TOPIC:-/mapir/indices/ndvi}"
FILTER_SET="${MAPIR_FILTER_SET:-OCN}"
OUTPUT_ENCODING="${MAPIR_NDVI_ENCODING:-32FC1}"
PUBLISH_COLOR="${MAPIR_PUBLISH_COLOR:-true}"
COLOR_TOPIC="${MAPIR_NDVI_COLOR_TOPIC:-/mapir/indices_color/ndvi}"
COLORMAP="${MAPIR_NDVI_COLORMAP:-plant_health}"
COLORIZE_MIN="${MAPIR_NDVI_COLORIZE_MIN:--1.0}"
COLORIZE_MAX="${MAPIR_NDVI_COLORIZE_MAX:-1.0}"
CUSTOM_COLORMAP="${MAPIR_NDVI_CUSTOM_COLORMAP:-}"
NIR_CHANNEL="${MAPIR_NIR_CHANNEL:--1}"
VISIBLE_CHANNEL="${MAPIR_VISIBLE_CHANNEL:--1}"
VISIBLE_BAND_NAME="${MAPIR_VISIBLE_BAND_NAME:-}"
EPS="${MAPIR_NDVI_EPS:-1.0e-6}"

cd "${SCRIPT_DIR}"
docker compose run --rm noetic bash -lc "
  set -euo pipefail
  source /opt/ros/noetic/setup.bash
  export PYTHONPATH=/workspace/fruc_ros_utils/src:/workspace/src:\${PYTHONPATH:-}
  mkdir -p \"\$(dirname \"${OUTPUT_BAG}\")\"
  ros1utils mapir_ndvi \
    --in \"${INPUT_BAG}\" \
    --out \"${OUTPUT_BAG}\" \
    --image-topic \"${IMAGE_TOPIC}\" \
    --output-topic \"${OUTPUT_TOPIC}\" \
    --output-encoding \"${OUTPUT_ENCODING}\" \
    $( [[ \"${PUBLISH_COLOR}\" == \"true\" ]] && printf '%s' '--publish-color' || printf '%s' '--no-publish-color' ) \
    --color-topic \"${COLOR_TOPIC}\" \
    --colormap \"${COLORMAP}\" \
    --colorize-min \"${COLORIZE_MIN}\" \
    --colorize-max \"${COLORIZE_MAX}\" \
    --custom-colormap \"${CUSTOM_COLORMAP}\" \
    --filter-set \"${FILTER_SET}\" \
    --nir-channel \"${NIR_CHANNEL}\" \
    --visible-channel \"${VISIBLE_CHANNEL}\" \
    --visible-band-name \"${VISIBLE_BAND_NAME}\" \
    --eps \"${EPS}\"
"
