#!/bin/bash
# batch_process_sessions.sh - Process multiple MCAP files in parallel using Docker
# Usage: ./batch_process_sessions.sh <raw_data_dir> <output_base_dir> [max_parallel] [docker_cpus_per_job]
#
# Output structure:
#   <raw_data_dir>/<basename>_processed.mcap (processed MCAP in same dir as original)
#   <output_base_dir>/<basename>.bag
#   <output_base_dir>/<basename>_processed.bag

set -e

RAW_DATA_DIR="$1"
OUTPUT_BASE_DIR="$2"
MAX_PARALLEL="${3:-2}"
DOCKER_CPUS="${4:-8}"

if [ -z "$RAW_DATA_DIR" ] || [ -z "$OUTPUT_BASE_DIR" ]; then
    echo "Usage: $0 <raw_data_dir> <output_base_dir> [max_parallel] [docker_cpus_per_job]"
    echo ""
    echo "Processes MCAP files in parallel Docker containers"
    echo "Output: processed MCAPs in <raw_data_dir>, bags in <output_base_dir>"
    echo ""
    echo "Example:"
    echo "  $0 /raw /output 2 8"
    exit 1
fi

if [ ! -d "$RAW_DATA_DIR" ]; then
    echo "[ERROR] Raw data directory not found: $RAW_DATA_DIR"
    exit 1
fi

mkdir -p "$OUTPUT_BASE_DIR"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROCESS_SCRIPT="$SCRIPT_DIR/process_session.sh"

if [ ! -f "$PROCESS_SCRIPT" ]; then
    echo "[ERROR] process_session.sh not found in: $SCRIPT_DIR"
    exit 1
fi

# Count input files
MCAP_COUNT=$(find "$RAW_DATA_DIR" -maxdepth 1 -name "*.mcap" -type f | wc -l)

if [ "$MCAP_COUNT" -eq 0 ]; then
    echo "[ERROR] No MCAP files found in: $RAW_DATA_DIR"
    exit 1
fi

# Calculate system resources and recommend MAX_PARALLEL
TOTAL_MEM_GB=$(($(grep MemTotal /proc/meminfo | awk '{print $2}') / 1024 / 1024))
RECOMMENDED_PARALLEL=$((TOTAL_MEM_GB / 50))
if [ "$RECOMMENDED_PARALLEL" -lt 1 ]; then
    RECOMMENDED_PARALLEL=1
fi
if [ "$RECOMMENDED_PARALLEL" -gt 4 ]; then
    RECOMMENDED_PARALLEL=4
fi

if [ "$MAX_PARALLEL" -gt "$RECOMMENDED_PARALLEL" ]; then
    echo "[WARNING] MAX_PARALLEL=$MAX_PARALLEL may cause resource exhaustion"
    echo "[INFO] System RAM: ${TOTAL_MEM_GB}GB → Recommended: $RECOMMENDED_PARALLEL"
    echo "[INFO] Reducing to $RECOMMENDED_PARALLEL"
    MAX_PARALLEL=$RECOMMENDED_PARALLEL
    echo ""
fi

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║ Batch Processing (Docker-based, Parallel)"
echo "║ Found $MCAP_COUNT MCAP files"
echo "║ Processing $MAX_PARALLEL in parallel"
echo "║ CPUs per session: $DOCKER_CPUS"
echo "║ System RAM: ${TOTAL_MEM_GB}GB"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

SUCCESSFUL=0
FAILED=0
FAILED_FILES=()

START_TIME=$(date +%s)

# Process files in parallel
find "$RAW_DATA_DIR" -maxdepth 1 -name "*.mcap" -type f -print0 | \
    sort -z | \
    xargs -0 -I {} -P "$MAX_PARALLEL" bash -c '
        mcap_file="$1"
        output_base="$2"
        process_script="$3"
        docker_cpus="$4"
        
        basename_val=$(basename "$mcap_file" .mcap)
        
        echo ""
        echo "╔════════════════════════════════════════════════════════════════╗"
        echo "║ Processing: $basename_val"
        echo "╚════════════════════════════════════════════════════════════════╝"
        
        if "$process_script" "$mcap_file" "$output_base" "$docker_cpus"; then
            echo "✅ SUCCESS: $basename_val"
        else
            echo "❌ FAILED: $basename_val"
            exit 1
        fi
    ' _ {} "$OUTPUT_BASE_DIR" "$PROCESS_SCRIPT" "$DOCKER_CPUS"

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║ Batch Processing Complete"
echo "║ Duration: $((DURATION / 60))m $((DURATION % 60))s"
echo "║ Total: $MCAP_COUNT files"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Output Structure:"
echo "  • Original MCAPs: $RAW_DATA_DIR/*.mcap"
echo "  • Processed MCAPs: $RAW_DATA_DIR/*_processed.mcap"
echo "  • ROS1 bags: $OUTPUT_BASE_DIR/*.bag"
echo "  • ROS1 bags (processed): $OUTPUT_BASE_DIR/*_processed.bag"
