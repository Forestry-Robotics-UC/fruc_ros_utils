#!/usr/bin/env bash
# Source this script to enable argcomplete for FRUC ROS utils commands.
# Example:
#   source scripts/enable_autocomplete.sh

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "Source this file instead of executing it:"
  echo "  source scripts/enable_autocomplete.sh"
  exit 1
fi

if ! command -v register-python-argcomplete >/dev/null 2>&1; then
  echo "register-python-argcomplete was not found."
  echo "Install argcomplete first (for example: pip install argcomplete)."
  return 1
fi

_fruc_completion_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "${_fruc_completion_repo_root}/scripts" ]; then
  export PATH="${_fruc_completion_repo_root}/scripts:${PATH}"
fi

for _fruc_cmd in bagutils ros1utils ros2utils; do
  if command -v "${_fruc_cmd}" >/dev/null 2>&1; then
    eval "$(register-python-argcomplete "${_fruc_cmd}")"
  fi
done

echo "FRUC ROS utils autocomplete enabled for: bagutils, ros1utils, ros2utils"

unset _fruc_cmd
unset _fruc_completion_repo_root
