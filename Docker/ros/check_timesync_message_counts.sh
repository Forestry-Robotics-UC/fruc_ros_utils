#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 /absolute/path/to/original_bag_dir [/absolute/path/to/processed_bag_dir]" >&2
    exit 1
fi

ORIG_DIR="$(realpath "$1")"
PROC_DIR="$(realpath "${2:-${ORIG_DIR}_Processed_0}")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "$ORIG_DIR" ]]; then
    echo "Original bag directory not found: $ORIG_DIR" >&2
    exit 1
fi

if [[ ! -d "$PROC_DIR" ]]; then
    echo "Processed bag directory not found: $PROC_DIR" >&2
    exit 1
fi

COMMON_ROOT="$(
python3 - "$ORIG_DIR" "$PROC_DIR" <<'PY'
import os
import sys

print(os.path.commonpath(sys.argv[1:]))
PY
)"

ORIG_REL="$(
python3 - "$ORIG_DIR" "$COMMON_ROOT" <<'PY'
import os
import sys

print(os.path.relpath(sys.argv[1], sys.argv[2]))
PY
)"

PROC_REL="$(
python3 - "$PROC_DIR" "$COMMON_ROOT" <<'PY'
import os
import sys

print(os.path.relpath(sys.argv[1], sys.argv[2]))
PY
)"

cd "$SCRIPT_DIR"
BAGS_PATH="$COMMON_ROOT" docker compose run --rm jazzy bash -lc "
  set -eo pipefail
  export AMENT_TRACE_SETUP_FILES=\${AMENT_TRACE_SETUP_FILES-}
  set +u
  source /opt/ros/jazzy/setup.bash
  source /opt/overlay_ws/install/setup.bash
  set -u
  python3 - '/bags/${ORIG_REL}' '/bags/${PROC_REL}' <<'PY'
import os
import sys
import yaml

TARGET_REMAP = {
    '/imu/data': '/imu/data/corrected',
    '/camera/color/image_raw': '/camera/color/image_raw/corrected',
    '/camera/aligned_depth_to_color/image_raw': '/camera/aligned_depth_to_color/image_raw/corrected',
}


def load_counts(bag_dir: str):
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.exists(meta_path):
        raise SystemExit(f'Metadata not found: {meta_path}. Run ros2 bag reindex first.')

    with open(meta_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    info = data.get('rosbag2_bagfile_information', {})
    topics = info.get('topics_with_message_count', [])
    counts = {}
    for entry in topics:
        topic_md = entry.get('topic_metadata', {})
        counts[topic_md.get('name')] = int(entry.get('message_count', 0))
    return counts, int(info.get('message_count', 0))


orig_dir, proc_dir = sys.argv[1], sys.argv[2]
orig_counts, orig_total = load_counts(orig_dir)
proc_counts, proc_total = load_counts(proc_dir)

errors = []

for orig_topic, orig_count in sorted(orig_counts.items()):
    proc_topic = TARGET_REMAP.get(orig_topic, orig_topic)
    proc_count = proc_counts.get(proc_topic)

    if proc_count is None:
        errors.append(f'Missing topic in processed bag: {proc_topic} (from {orig_topic})')
        continue

    if proc_count != orig_count:
        errors.append(
            f'Count mismatch for {orig_topic} -> {proc_topic}: '
            f'original={orig_count}, processed={proc_count}'
        )

if orig_total != proc_total:
    errors.append(f'Total message count mismatch: original={orig_total}, processed={proc_total}')

print(f'Original total messages:  {orig_total}')
print(f'Processed total messages: {proc_total}')
print(f'Original topics:          {len(orig_counts)}')
print(f'Processed topics:         {len(proc_counts)}')

if errors:
    print('')
    print('Count check failed:')
    for err in errors:
        print(f'  - {err}')
    raise SystemExit(1)

print('')
print('Count check passed: all source topics are present in the processed bag with matching message counts.')
PY
"
