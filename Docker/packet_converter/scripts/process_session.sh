#!/bin/bash
# process_session.sh - Docker-based packet-to-points workflow
# Usage: ./process_session.sh <raw_mcap_path> <output_base_dir> [docker_cpus]

set -e

RAW_MCAP="$1"
OUTPUT_BASE_DIR="$2"
DOCKER_CPUS="${3:-8}"

if [ -z "$RAW_MCAP" ] || [ -z "$OUTPUT_BASE_DIR" ]; then
    echo "Usage: $0 <raw_mcap_path> <output_base_dir> [docker_cpus]"
    exit 1
fi

if [ ! -f "$RAW_MCAP" ]; then
    echo "[ERROR] MCAP file not found: $RAW_MCAP"
    exit 1
fi

INPUT_DIR="$(dirname "$RAW_MCAP")"
BASENAME=$(basename "$RAW_MCAP" .mcap)
mkdir -p "$OUTPUT_BASE_DIR"
mkdir -p "$INPUT_DIR/logs"

LOG_FILE="$INPUT_DIR/logs/${BASENAME}_conversion.log"
MCAP_SIZE=$(du -h "$RAW_MCAP" | cut -f1)
MCAP_SIZE_BYTES=$(du -b "$RAW_MCAP" | cut -f1)

REQUIRED_SPACE_BYTES=$((MCAP_SIZE_BYTES * 4))
AVAILABLE_SPACE_BYTES=$(df "$OUTPUT_BASE_DIR" | tail -1 | awk '{print $4 * 1024}')

if [ "$AVAILABLE_SPACE_BYTES" -lt "$REQUIRED_SPACE_BYTES" ]; then
    echo "[ERROR] Insufficient disk space!"
    exit 1
fi

TOTAL_MEM_MB=$(($(grep MemTotal /proc/meminfo | awk '{print $2}') / 1024))
DOCKER_MEMORY_MB=$((TOTAL_MEM_MB * 80 / 100))
if [ "$DOCKER_MEMORY_MB" -gt 65536 ]; then
    DOCKER_MEMORY_MB=65536
fi

CONVERTER_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Check Docker image exists
DOCKER_IMAGE="ros2-apparatus-converter:jazzy"
if ! docker image inspect "$DOCKER_IMAGE" &>/dev/null; then
    echo "[ERROR] Docker image not found: $DOCKER_IMAGE"
    echo "Build with: cd $CONVERTER_DIR && docker compose build"
    exit 1
fi

log_msg() {
    local ts=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$ts] $@"
}

cleanup() {
    local code=$?
    if [ $code -ne 0 ]; then
        log_msg "[CLEANUP] Conversion failed"
        PROCESSED_MCAP_DIR="$INPUT_DIR/${BASENAME}_processed.mcap"
        if [ -d "$PROCESSED_MCAP_DIR" ]; then
            size=$(du -bs "$PROCESSED_MCAP_DIR" 2>/dev/null | cut -f1 || echo "0")
            if [ "$size" -lt $((MCAP_SIZE_BYTES / 10)) ]; then
                chmod -R u+w "$PROCESSED_MCAP_DIR" 2>/dev/null || true
                rm -rf "$PROCESSED_MCAP_DIR" 2>/dev/null || true
            fi
        fi
    fi
    # Fix permissions on output from Docker (created as root)
    chmod -R u+w "$INPUT_DIR/${BASENAME}_processed.mcap" 2>/dev/null || true
    exit $code
}
trap cleanup EXIT INT TERM

log_msg "=============================================="
log_msg "Docker-based Conversion: $BASENAME"
log_msg "=============================================="
log_msg "Input MCAP: $RAW_MCAP ($MCAP_SIZE)"
log_msg "Docker Memory: ${DOCKER_MEMORY_MB}MB"

# Auto-detect Ouster metadata file in input directory
METADATA_ARGS=""
METADATA_FILE=$(find "$INPUT_DIR" -maxdepth 1 -name "*ouster*metadata*" -o -name "*metadata*.txt" -o -name "*metadata*.json" 2>/dev/null | head -1)
if [ -n "$METADATA_FILE" ]; then
    METADATA_BASENAME=$(basename "$METADATA_FILE")
    METADATA_ARGS="--metadata-file /work/input/$METADATA_BASENAME"
    log_msg "Ouster metadata: $METADATA_FILE"
else
    log_msg "No metadata file found, will use /ouster/metadata topic from MCAP"
fi
echo ""

log_msg "STEP 1: Decoding packets..."
PROCESSED_MCAP_DIR="$INPUT_DIR/${BASENAME}_processed.mcap"
START=$(date +%s)

# Remove stale directory if it exists
rm -rf "$PROCESSED_MCAP_DIR"

docker run --rm \
    --memory="${DOCKER_MEMORY_MB}m" \
    --user $(id -u):$(id -g) \
    -v "$CONVERTER_DIR/scripts:/app/scripts:ro" \
    -v "$INPUT_DIR:/work/input:ro" \
    -v "$INPUT_DIR:/work/output" \
    "ros2-apparatus-converter:jazzy" \
    -i /work/input/"${BASENAME}".mcap -o /work/output/"${BASENAME}"_processed.mcap --output-format mcap --skip-original-topics $METADATA_ARGS || {
    log_msg "FAIL: Decode failed"
    exit 1
}

END=$(date +%s)
if [ -d "$PROCESSED_MCAP_DIR" ]; then
    chmod -R u+w "$PROCESSED_MCAP_DIR" 2>/dev/null || true
    size=$(du -h "$PROCESSED_MCAP_DIR" | cut -f1)
    log_msg "OK: $size ($((END - START))s)"
else
    log_msg "FAIL: Processed MCAP not found at $PROCESSED_MCAP_DIR"
    exit 1
fi
echo ""

log_msg "STEP 2: Converting to ROS1 bags..."
ORIG_BAG="$OUTPUT_BASE_DIR/${BASENAME}.bag"
PROC_BAG="$OUTPUT_BASE_DIR/${BASENAME}_processed.bag"
START=$(date +%s)

# Convert original MCAP to bag (folder mode with --only to pick this one file)
docker run --rm \
    --memory="${DOCKER_MEMORY_MB}m" \
    --user $(id -u):$(id -g) \
    -v "$CONVERTER_DIR/scripts:/app/scripts:ro" \
    -v "$INPUT_DIR:/work/input:ro" \
    -v "$OUTPUT_BASE_DIR:/work/output" \
    --entrypoint bash \
    "ros2-apparatus-converter:jazzy" \
    -c "source /app/.venv/bin/activate 2>/dev/null; python3 /app/scripts/mcap_to_ros1.py /work/input /work/output --only ${BASENAME}.mcap" || {
    log_msg "FAIL: Original conversion failed"
    exit 1
}
if [ -f "$ORIG_BAG" ]; then
    log_msg "OK: Original bag: $(du -h "$ORIG_BAG" | cut -f1)"
else
    log_msg "FAIL: Original bag not found at $ORIG_BAG"
    exit 1
fi

# Convert processed MCAP to bag (already a rosbag2 directory with metadata.yaml)
docker run --rm \
    --memory="${DOCKER_MEMORY_MB}m" \
    --user $(id -u):$(id -g) \
    -v "$CONVERTER_DIR/scripts:/app/scripts:ro" \
    -v "$INPUT_DIR:/work/input:ro" \
    -v "$OUTPUT_BASE_DIR:/work/output" \
    --entrypoint bash \
    "ros2-apparatus-converter:jazzy" \
    -c "source /app/.venv/bin/activate 2>/dev/null; python3 /app/scripts/mcap_to_ros1.py /work/input/${BASENAME}_processed.mcap /work/output" || {
    log_msg "FAIL: Processed conversion failed"
    exit 1
}
if [ -f "$PROC_BAG" ]; then
    log_msg "OK: Processed bag: $(du -h "$PROC_BAG" | cut -f1)"
else
    log_msg "FAIL: Processed bag not found at $PROC_BAG"
    exit 1
fi

END=$(date +%s)
log_msg "=============================================="
log_msg "OK: Conversion Complete ($((END - START))s)"
log_msg "=============================================="
log_msg "Processed: $PROCESSED_MCAP_DIR"
log_msg "Bags: $OUTPUT_BASE_DIR/"
