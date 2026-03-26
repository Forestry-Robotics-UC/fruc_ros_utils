#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# iKalibr Docker helper
# Usage:  ./docker/run.sh [solver|solver-cuda|imu-calib|shell]
# ─────────────────────────────────────────────────────────────────────────────
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

SERVICE="${1:-solver}"

# If this script was invoked with `sh run.sh` (dash), re-exec under bash
if [ -z "${BASH_VERSION:-}" ]; then
  exec bash "$0" "$@"
fi

# If running as root via sudo, prefer the original user's HOME for compose
# variable substitution so ${HOME}/.Xauthority refers to the calling user.
if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  HOST_HOME=$(eval echo "~${SUDO_USER}")
  HOST_UID="$(id -u "${SUDO_USER}")"
  HOST_GID="$(id -g "${SUDO_USER}")"
else
  HOST_HOME="${HOME}"
  HOST_UID="$(id -u)"
  HOST_GID="$(id -g)"
fi

if [ -z "${BAGS_PATH:-}" ]; then
  echo "[iKalibr] BAGS_PATH is not set."
  echo "Set it in the current shell or preserve it through sudo."
  echo "Examples:"
  echo "  BAGS_PATH=/mnt/t7_shield ./run.sh solver-cuda"
  echo "  sudo --preserve-env=BAGS_PATH,XAUTHORITY,DISPLAY ./run.sh solver-cuda"
  echo "  sudo BAGS_PATH=/mnt/t7_shield XAUTHORITY=${XAUTHORITY:-\$XAUTHORITY} DISPLAY=${DISPLAY:-\$DISPLAY} ./run.sh solver-cuda"
  exit 1
fi

ensure_host_mount_ownership() {
  mkdir -p "${SCRIPT_DIR}/output"

  if [ "$(id -u)" -eq 0 ]; then
    chown -R "${HOST_UID}:${HOST_GID}" "${SCRIPT_DIR}/config" "${SCRIPT_DIR}/output"
    return
  fi

  if [ ! -w "${SCRIPT_DIR}/output" ] || [ ! -w "${SCRIPT_DIR}/config" ]; then
    echo "[iKalibr] Warning: config/ or output/ is not writable by uid ${HOST_UID}:${HOST_GID}."
    echo "[iKalibr] Re-run with sudo once so the helper can repair ownership automatically."
  fi
}

ensure_nvidia_persistenced() {
  local socket_path="/run/nvidia-persistenced/socket"
  local daemon_user="nvidia-persistenced"

  if [ -S "${socket_path}" ]; then
    return
  fi

  if ! command -v nvidia-persistenced >/dev/null 2>&1; then
    echo "[iKalibr] Warning: nvidia-persistenced is not installed on the host."
    return
  fi

  if [ "$(id -u)" -ne 0 ]; then
    echo "[iKalibr] Warning: ${socket_path} is missing and starting nvidia-persistenced requires sudo."
    return
  fi

  mkdir -p /run/nvidia-persistenced
  if id -u "${daemon_user}" >/dev/null 2>&1; then
    chown "${daemon_user}:${daemon_user}" /run/nvidia-persistenced
    nvidia-persistenced --user "${daemon_user}" --no-persistence-mode --verbose >/tmp/fruc-nvidia-persistenced.log 2>&1 || true
  else
    nvidia-persistenced --no-persistence-mode --verbose >/tmp/fruc-nvidia-persistenced.log 2>&1 || true
  fi

  sleep 1

  if [ ! -S "${socket_path}" ]; then
    echo "[iKalibr] Warning: failed to create ${socket_path}."
    echo "[iKalibr] Check /tmp/fruc-nvidia-persistenced.log on the host."
  fi
}

# Helper to run docker compose with HOME set to the host user's home
dc() {
  local host_xauthority="${HOST_HOME}/.Xauthority"
  local session_xauthority="${XAUTHORITY:-}"
  local xauthority_path=""

  if [ -n "${session_xauthority}" ] && [ -f "${session_xauthority}" ]; then
    xauthority_path="${session_xauthority}"
  elif [ -f "${host_xauthority}" ]; then
    xauthority_path="${host_xauthority}"
  else
    xauthority_path="$(mktemp /tmp/fruc-docker-empty.XXXXXX.xauth)"
    temp_xauthority_path="${xauthority_path}"
  fi

  env HOME="${HOST_HOME}" \
    XAUTHORITY_PATH="${xauthority_path}" \
    LOCAL_UID="${HOST_UID}" \
    LOCAL_GID="${HOST_GID}" \
    docker compose -f "${COMPOSE_FILE}" "$@"
}

cleanup_runtime_artifacts() {
  if [ "${xhost_granted:-0}" -eq 1 ]; then
    xhost -si:localuser:root >/dev/null 2>&1 || true
  fi
  if [ -n "${temp_xauthority_path:-}" ] && [ -f "${temp_xauthority_path}" ]; then
    rm -f "${temp_xauthority_path}" >/dev/null 2>&1 || true
  fi
}

xhost_granted=0
temp_xauthority_path=""
if [ "${ALLOW_XHOST_ROOT:-0}" = "1" ]; then
  # xhost cannot target a Docker container name; it can only grant to a local
  # user on the host. Keep this opt-in and revoke it on exit.
  xhost +si:localuser:root >/dev/null 2>&1 || true
  xhost_granted=1
fi
trap cleanup_runtime_artifacts EXIT

if [ "${SERVICE}" = "solver-cuda" ]; then
  if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
    echo "[iKalibr] Warning: host nvidia-smi is not healthy."
    echo "[iKalibr] Trying the GPU container anyway because solver-cuda was requested explicitly."
  fi
fi

if [ "${SERVICE}" = "solver-cuda" ] || [ "${SERVICE}" = "shell" ]; then
  ensure_nvidia_persistenced
fi

ensure_host_mount_ownership

case "$SERVICE" in
  solver|solver-cuda|imu-calib|shell)
    echo "[iKalibr] Starting service: $SERVICE"
    dc run --rm "$SERVICE"
    ;;
  pull)
    echo "[iKalibr] Pulling latest image..."
    dc pull
    ;;
  *)
    echo "Usage: $0 [solver|solver-cuda|imu-calib|shell|pull]"
    echo ""
    echo "  solver       – Run the main spatiotemporal calibration on CPU"
    echo "  solver-cuda  – Run the main spatiotemporal calibration with GPU access"
    echo "  imu-calib    – Run IMU intrinsics pre-calibration"
    echo "  shell        – Open an interactive bash shell inside the container"
    echo "  pull         – Pull / update the Docker image"
    exit 1
    ;;
esac
