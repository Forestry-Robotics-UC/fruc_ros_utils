#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

BUILD_JOBS="$(nproc)"
if (( BUILD_JOBS > 4 )); then
  BUILD_JOBS=4
fi

echo "[iKalibr] Rebuilding COLMAP, PoseLib, and GLOMAP with CUDA support."
echo "[iKalibr] Build jobs: ${BUILD_JOBS}"

ensure_openimageio_cmake_config() {
  local existing_config=""
  local include_dir=""
  local oiio_lib=""
  local oiio_util_lib=""
  local config_dir="/usr/local/lib/cmake/OpenImageIO"

  existing_config="$(find /usr /usr/local -type f -name 'OpenImageIOConfig.cmake' 2>/dev/null | head -n 1 || true)"
  if [[ -n "${existing_config}" ]]; then
    echo "[iKalibr] Found existing OpenImageIO CMake config: ${existing_config}"
    return
  fi

  include_dir="$(find /usr/include /usr/local/include -path '*/OpenImageIO/imageio.h' 2>/dev/null | head -n 1 || true)"
  if [[ -n "${include_dir}" ]]; then
    include_dir="$(dirname "$(dirname "${include_dir}")")"
  fi

  oiio_lib="$(find /usr/lib /usr/lib64 /usr/lib/x86_64-linux-gnu /usr/local/lib -name 'libOpenImageIO.so*' 2>/dev/null | head -n 1 || true)"
  oiio_util_lib="$(find /usr/lib /usr/lib64 /usr/lib/x86_64-linux-gnu /usr/local/lib -name 'libOpenImageIO_Util.so*' 2>/dev/null | head -n 1 || true)"

  if [[ -z "${include_dir}" || -z "${oiio_lib}" ]]; then
    echo "[iKalibr] OpenImageIO headers or libraries were not found after apt install." >&2
    return 1
  fi

  mkdir -p "${config_dir}"
  cat > "${config_dir}/OpenImageIOConfig.cmake" <<EOF
set(OpenImageIO_FOUND TRUE)
set(OPENIMAGEIO_FOUND TRUE)
set(OpenImageIO_INCLUDE_DIR "${include_dir}")
set(OPENIMAGEIO_INCLUDE_DIR "${include_dir}")
set(OpenImageIO_INCLUDE_DIRS "${include_dir}")
set(OPENIMAGEIO_INCLUDE_DIRS "${include_dir}")
set(OpenImageIO_LIBRARIES "${oiio_lib}")
set(OPENIMAGEIO_LIBRARIES "${oiio_lib}")

if(NOT TARGET OpenImageIO::OpenImageIO)
  add_library(OpenImageIO::OpenImageIO SHARED IMPORTED)
  set_target_properties(OpenImageIO::OpenImageIO PROPERTIES
    IMPORTED_LOCATION "${oiio_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
  )
endif()
EOF

  if [[ -n "${oiio_util_lib}" ]]; then
    cat >> "${config_dir}/OpenImageIOConfig.cmake" <<EOF
list(APPEND OpenImageIO_LIBRARIES "${oiio_util_lib}")
list(APPEND OPENIMAGEIO_LIBRARIES "${oiio_util_lib}")

if(NOT TARGET OpenImageIO::OpenImageIO_Util)
  add_library(OpenImageIO::OpenImageIO_Util SHARED IMPORTED)
  set_target_properties(OpenImageIO::OpenImageIO_Util PROPERTIES
    IMPORTED_LOCATION "${oiio_util_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
  )
endif()
EOF
  fi

  cat > "${config_dir}/OpenImageIOConfigVersion.cmake" <<'EOF'
set(PACKAGE_VERSION "2")
if(PACKAGE_FIND_VERSION)
  if(PACKAGE_FIND_VERSION_MAJOR EQUAL 2)
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    set(PACKAGE_VERSION_EXACT FALSE)
  else()
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
  endif()
endif()
EOF

  echo "[iKalibr] Generated compatibility OpenImageIO CMake config at ${config_dir}"
}

patch_colmap_for_legacy_cuda() {
  local nvcc_release=""
  local nvcc_major=""
  local cmake_file="/tmp/colmap-src/CMakeLists.txt"

  if ! command -v nvcc >/dev/null 2>&1; then
    return
  fi

  nvcc_release="$(nvcc --version | sed -n 's/.*release \([0-9][0-9]*\)\..*/\1/p' | tail -n 1)"
  if [[ -z "${nvcc_release}" ]]; then
    return
  fi
  nvcc_major="${nvcc_release}"

  if (( nvcc_major >= 11 )); then
    return
  fi

  if [[ ! -f "${cmake_file}" ]]; then
    return
  fi

  echo "[iKalibr] Detected nvcc ${nvcc_major}.x; patching COLMAP CUDA standard from 17 to 14 for compatibility."
  sed -i 's/set(CMAKE_CUDA_STANDARD 17)/set(CMAKE_CUDA_STANDARD 14)/' "${cmake_file}"
}

normalize_cuda_architectures() {
  if [[ -z "${COLMAP_CUDA_ARCHITECTURES:-}" ]]; then
    COLMAP_CUDA_ARCHITECTURES="all-major"
    return
  fi

  if [[ "${COLMAP_CUDA_ARCHITECTURES}" != "native" ]]; then
    return
  fi

  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
    return
  fi

  echo "[iKalibr] COLMAP_CUDA_ARCHITECTURES=native is not usable during a headless Docker build."
  echo "[iKalibr] Falling back to all-major."
  COLMAP_CUDA_ARCHITECTURES="all-major"
}

apt-get update
apt-get install -y --no-install-recommends \
  ca-certificates \
  build-essential \
  cmake \
  git \
  ninja-build \
  pkg-config \
  libatlas-base-dev \
  libboost-filesystem-dev \
  libboost-graph-dev \
  libboost-program-options-dev \
  libboost-system-dev \
  libceres-dev \
  libcgal-dev \
  libcurl4-openssl-dev \
  libeigen3-dev \
  libflann-dev \
  libfreeimage-dev \
  libgflags-dev \
  libglew-dev \
  libgoogle-glog-dev \
  libmetis-dev \
  libopenimageio-dev \
  libssl-dev \
  libsqlite3-dev \
  libsuitesparse-dev \
  openimageio-tools \
  qtbase5-dev \
  libqt5opengl5-dev

ensure_openimageio_cmake_config

if ! command -v nvcc >/dev/null 2>&1; then
  if ! apt-get install -y --no-install-recommends nvidia-cuda-toolkit; then
    echo "[iKalibr] Failed to install CUDA toolkit for the COLMAP/GLOMAP rebuild." >&2
    exit 1
  fi
fi

mkdir -p /usr/include/opencv4

COLMAP_GIT_REF="${COLMAP_GIT_REF:-3.13.0}"
GLOMAP_GIT_REF="${GLOMAP_GIT_REF:-1.2.0}"
POSELIB_GIT_REF="${POSELIB_GIT_REF:-master}"
IKALIBR_BUILD_GLOMAP="${IKALIBR_BUILD_GLOMAP:-0}"
normalize_cuda_architectures

colmap_cuda_args=()
if [[ -n "${COLMAP_CUDA_ARCHITECTURES:-}" ]]; then
  colmap_cuda_args+=("-DCMAKE_CUDA_ARCHITECTURES=${COLMAP_CUDA_ARCHITECTURES}")
fi

rm -rf /tmp/colmap-src
git clone --recursive --depth 1 --branch "${COLMAP_GIT_REF}" https://github.com/colmap/colmap.git /tmp/colmap-src
patch_colmap_for_legacy_cuda
cmake -S /tmp/colmap-src -B /tmp/colmap-src/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCUDA_ENABLED=ON \
  "${colmap_cuda_args[@]}"
cmake --build /tmp/colmap-src/build -j"${BUILD_JOBS}"
cmake --install /tmp/colmap-src/build
ldconfig

if [[ "${IKALIBR_BUILD_GLOMAP}" == "1" ]]; then
  echo "[iKalibr] IKALIBR_BUILD_GLOMAP=1: building PoseLib + GLOMAP."
  rm -rf /tmp/PoseLib-src
  if ! git clone --recursive --depth 1 --branch "${POSELIB_GIT_REF}" https://github.com/PoseLib/PoseLib.git /tmp/PoseLib-src; then
    rm -rf /tmp/PoseLib-src
    if [[ "${POSELIB_GIT_REF}" != "master" ]]; then
      echo "[iKalibr] PoseLib ref '${POSELIB_GIT_REF}' was not found. Retrying with 'master'."
      git clone --recursive --depth 1 --branch master https://github.com/PoseLib/PoseLib.git /tmp/PoseLib-src
    else
      echo "[iKalibr] PoseLib ref '${POSELIB_GIT_REF}' was not found. Retrying with repository default branch."
      git clone --recursive --depth 1 https://github.com/PoseLib/PoseLib.git /tmp/PoseLib-src
    fi
  fi
  cmake -S /tmp/PoseLib-src -B /tmp/PoseLib-src/build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_TESTING=OFF
  cmake --build /tmp/PoseLib-src/build -j"${BUILD_JOBS}"
  cmake --install /tmp/PoseLib-src/build
  ldconfig

  colmap_config_file="$(find /usr/local -type f \( -name 'colmap-config.cmake' -o -name 'COLMAPConfig.cmake' \) | head -n 1)"
  poselib_config_file="$(find /usr/local -type f \( -name 'PoseLibConfig.cmake' -o -name 'poselib-config.cmake' \) | head -n 1)"

  glomap_args=(
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=/usr/local
    -DFETCH_COLMAP=OFF
    -DFETCH_POSELIB=OFF
  )

  if [[ -n "${colmap_config_file}" ]]; then
    glomap_args+=("-Dcolmap_DIR=$(dirname "${colmap_config_file}")")
    glomap_args+=("-DCOLMAP_DIR=$(dirname "${colmap_config_file}")")
  fi

  if [[ -n "${poselib_config_file}" ]]; then
    glomap_args+=("-DPoseLib_DIR=$(dirname "${poselib_config_file}")")
  fi

  rm -rf /tmp/glomap-src
  git clone --recursive --depth 1 --branch "${GLOMAP_GIT_REF}" https://github.com/colmap/glomap.git /tmp/glomap-src
  cmake -S /tmp/glomap-src -B /tmp/glomap-src/build -G Ninja "${glomap_args[@]}"
  cmake --build /tmp/glomap-src/build -j"${BUILD_JOBS}"
  cmake --install /tmp/glomap-src/build
  ldconfig
else
  echo "[iKalibr] IKALIBR_BUILD_GLOMAP=0: skipping PoseLib/GLOMAP and using COLMAP mapper path."
fi

rm -rf /tmp/colmap-src /tmp/PoseLib-src /tmp/glomap-src /var/lib/apt/lists/*
