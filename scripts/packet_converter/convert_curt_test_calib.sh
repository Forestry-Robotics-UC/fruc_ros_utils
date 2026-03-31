#!/bin/bash
# convert_curt_test_calib.sh - Convert curt_test_calib dataset (jazzy_no_post)
# Output: processed MCAPs in jazzy_no_post, ROS1 bags in ../noetic_ros
# Usage: ./convert_curt_test_calib.sh [max_parallel_jobs] [cpus_per_job]

set -e

# Dataset configuration
INPUT_DIR="/mnt/phd/datasets/curt_test_calib/jazzy_no_post"
OUTPUT_DIR="/mnt/phd/datasets/curt_test_calib/noetic_ros"

# Job parameters (optional overrides)
MAX_PARALLEL="${1:-2}"
DOCKER_CPUS="${2:-8}"

# Validate input directory exists
if [ ! -d "$INPUT_DIR" ]; then
    echo "[ERROR] Input directory not found: $INPUT_DIR"
    exit 1
fi

# Find MCAP files
MCAP_FILES=$(find "$INPUT_DIR" -maxdepth 1 -name "*.mcap" -type f)
MCAP_COUNT=$(echo "$MCAP_FILES" | wc -w)

if [ "$MCAP_COUNT" -eq 0 ]; then
    echo "[ERROR] No MCAP files found in: $INPUT_DIR"
    echo "       Expected: $INPUT_DIR/*.mcap"
    exit 1
fi

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║ curt_test_calib Dataset Conversion Setup"
echo "╠════════════════════════════════════════════════════════════════╣"
echo "║ Input:  $INPUT_DIR"
echo "║ Output: $OUTPUT_DIR"
echo "║ Files found: $MCAP_COUNT MCAP files"
echo "║ Parallel jobs: $MAX_PARALLEL"
echo "║ CPUs per job: $DOCKER_CPUS"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# List files to be processed
echo "Files to process:"
echo "$MCAP_FILES" | while read -r f; do
    if [ -n "$f" ]; then
        size=$(du -h "$f" | cut -f1)
        echo "  • $(basename "$f") ($size)"
    fi
done
echo ""

# Confirm before proceeding
read -p "Proceed with conversion? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo "Starting conversion..."
echo ""

# Get the batch processing script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_SCRIPT="$SCRIPT_DIR/batch_process_sessions.sh"

if [ ! -f "$BATCH_SCRIPT" ]; then
    echo "[ERROR] batch_process_sessions.sh not found in: $SCRIPT_DIR"
    exit 1
fi

# Run batch processor
"$BATCH_SCRIPT" "$INPUT_DIR" "$OUTPUT_DIR" "$MAX_PARALLEL" "$DOCKER_CPUS"

echo ""
echo "✅ Conversion complete!"
echo ""
echo "Output structure:"
echo "  • Original MCAPs: $INPUT_DIR/*.mcap"
echo "  • Processed MCAPs: $INPUT_DIR/*_processed.mcap"
echo "  • ROS1 bags (original): $OUTPUT_DIR/*.bag"
echo "  • ROS1 bags (processed → iKalibr): $OUTPUT_DIR/*_processed.bag"
