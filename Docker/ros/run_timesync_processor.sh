#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 /absolute/path/to/bag_dir [60-80 memory percent]" >&2
    exit 1
fi

BAG_DIR="$(realpath "$1")"
MEMORY_PERCENT="${2:-${COMPOSE_MEMORY_PERCENT:-70}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
TIMESYNC_ROOT="$(realpath "${REPO_ROOT}/../rosbag_timesync_utils")"

if [[ ! -d "$BAG_DIR" ]]; then
    echo "Bag directory not found: $BAG_DIR" >&2
    exit 1
fi

if [[ ! -d "$TIMESYNC_ROOT" ]]; then
    echo "Timesync repo not found: $TIMESYNC_ROOT" >&2
    exit 1
fi

BAGS_ROOT="$(dirname "$BAG_DIR")"
BAG_NAME="$(basename "$BAG_DIR")"

bash "${DOCKER_ROOT}/apply_compose_memory_limits.sh" "$MEMORY_PERCENT"

cd "$SCRIPT_DIR"
BAGS_PATH="$BAGS_ROOT" docker compose run --rm \
  -v "${TIMESYNC_ROOT}:/timesync:ro" \
  jazzy bash -lc "
    set -eo pipefail
    export AMENT_TRACE_SETUP_FILES=\${AMENT_TRACE_SETUP_FILES-}
    set +u
    source /opt/ros/jazzy/setup.bash
    source /opt/overlay_ws/install/setup.bash
    set -u
    cp /timesync/Scripts/Bag_Processing/bag_processor.py /tmp/bag_processor.py
    sed -i 's/# id=topic_info.id,/id=topic_info.id,/' /tmp/bag_processor.py
    cd /
    python3 /tmp/bag_processor.py bags/${BAG_NAME}
    ros2 bag reindex /bags/${BAG_NAME}_Processed_0
  "
