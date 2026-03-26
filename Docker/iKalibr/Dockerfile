FROM ulong2/ie_kalibr_image:latest

ARG LOCAL_UID=1000
ARG LOCAL_GID=1000
ARG IKALIBR_REBUILD_CUDA=0
ARG IKALIBR_REBUILD_SFM_CUDA=0
ARG IKALIBR_REBUILD_SRC=0
ARG IKALIBR_GIT_REF=master
ARG CERES_GIT_REF=2.2.0
ARG COLMAP_GIT_REF=3.13.0
ARG GLOMAP_GIT_REF=1.2.0
ARG POSELIB_GIT_REF=master
ARG COLMAP_CUDA_ARCHITECTURES=all-major
ARG IKALIBR_BUILD_GLOMAP=0

SHELL ["/bin/bash", "-lc"]

COPY scripts/install_sfm_cuda.sh /tmp/install_sfm_cuda.sh

# Optional CUDA rebuild path:
#   IKALIBR_REBUILD_CUDA=1 rebuilds Ceres with CUDA support.
# Notes:
# - Requires internet access during docker build.
# - Enabling CUDA solving in iKalibr also requires rebuilding the iKalibr
#   libraries from source because the base image statically embeds Ceres into
#   libikalibr_solver.so.
# Split into two RUN commands so the Ceres CUDA layer is cached independently.

# --- Step 1: Ceres with CUDA ---
RUN if [[ "${IKALIBR_REBUILD_CUDA}" == "1" ]]; then \
            set -euo pipefail; \
            export DEBIAN_FRONTEND=noninteractive; \
            apt-get update; \
            apt-get install -y --no-install-recommends \
                git build-essential ninja-build pkg-config \
                libatlas-base-dev libsuitesparse-dev \
                libgoogle-glog-dev libgflags-dev libeigen3-dev \
                wget ca-certificates gnupg; \
            \
            # Ceres 2.2.0 CUDA kernels require C++17 device code (CUDA 11+). \
            # The base image ships CUDA 10.1 which only supports C++14. \
            # Install CUDA 11.8 toolkit from NVIDIA repos for the build. \
            NVCC_VER=$(/usr/bin/nvcc --version 2>/dev/null | grep -oP 'release \K[0-9]+' || echo 0); \
            if (( NVCC_VER < 11 )); then \
                echo "[iKalibr] CUDA ${NVCC_VER}.x too old for Ceres CUDA – installing CUDA 11.8 toolkit"; \
                wget -qO /tmp/cuda-keyring.deb \
                    https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb; \
                dpkg -i /tmp/cuda-keyring.deb; \
                rm /tmp/cuda-keyring.deb; \
                apt-get update; \
                apt-get install -y --no-install-recommends cuda-toolkit-11-8; \
            fi; \
            rm -rf /var/lib/apt/lists/*; \
            \
            # Use CUDA 11.8 nvcc if installed, else the existing one. \
            if [[ -x /usr/local/cuda-11.8/bin/nvcc ]]; then \
                export PATH=/usr/local/cuda-11.8/bin:$PATH; \
                export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH:-}; \
            fi; \
            echo "[iKalibr] Using nvcc: $(nvcc --version | tail -1)"; \
            \
            BUILD_JOBS=$(nproc); \
            if (( BUILD_JOBS > 4 )); then BUILD_JOBS=4; fi; \
            echo "[iKalibr] Build jobs: ${BUILD_JOBS}"; \
            \
            rm -rf /tmp/ceres-src; \
            git clone --depth 1 --branch "${CERES_GIT_REF}" https://github.com/ceres-solver/ceres-solver.git /tmp/ceres-src; \
            cmake -S /tmp/ceres-src -B /tmp/ceres-src/build -G Ninja \
                -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_SHARED_LIBS=ON \
                -DBUILD_TESTING=OFF \
                -DBUILD_EXAMPLES=OFF \
                -DBUILD_BENCHMARKS=OFF \
                -DUSE_CUDA=ON; \
            cmake --build /tmp/ceres-src/build -j"${BUILD_JOBS}"; \
            cmake --install /tmp/ceres-src/build; \
            ldconfig; \
            rm -rf /tmp/ceres-src; \
        fi

# --- Step 2 (optional): Rebuild iKalibr from source ---
# Required when using a newly rebuilt Ceres because the shipped iKalibr
# libraries in the base image statically embed Ceres.
RUN if [[ "${IKALIBR_REBUILD_SRC}" == "1" ]]; then \
            set -euo pipefail; \
            BUILD_JOBS=$(nproc); \
            if (( BUILD_JOBS > 4 )); then BUILD_JOBS=4; fi; \
            \
            for attempt in 1 2 3; do \
                rm -rf /tmp/iKalibr-src; \
                if git clone --recursive --depth 1 --branch "${IKALIBR_GIT_REF}" https://github.com/Unsigned-Long/iKalibr.git /tmp/iKalibr-src; then \
                    break; \
                fi; \
                if (( attempt == 3 )); then \
                    echo "[iKalibr] Failed to clone source after ${attempt} attempts." >&2; \
                    exit 1; \
                fi; \
                echo "[iKalibr] Clone attempt ${attempt} failed; retrying..."; \
                sleep $((attempt * 5)); \
            done; \
            cd /tmp/iKalibr-src; \
            chmod +x build_thirdparty.sh; \
            ./build_thirdparty.sh; \
            source /opt/ros/noetic/setup.bash; \
            mkdir -p /tmp/iKalibr-ws/src; \
            ln -s /tmp/iKalibr-src /tmp/iKalibr-ws/src/ikalibr; \
            cd /tmp/iKalibr-ws; \
            catkin_make install -DCMAKE_BUILD_TYPE=Release -j"${BUILD_JOBS}"; \
            rm -rf /home/iKalibr/install; \
            cp -a /tmp/iKalibr-ws/install /home/iKalibr/install; \
            ldconfig; \
            rm -rf /tmp/iKalibr-src /tmp/iKalibr-ws; \
        fi

# Optional SfM CUDA rebuild path:
#   IKALIBR_REBUILD_SFM_CUDA=1 will rebuild COLMAP with CUDA from source.
#   IKALIBR_BUILD_GLOMAP=1 also rebuilds PoseLib + GLOMAP (off by default).
# Notes:
# - Requires internet access during docker build.
# - This path can take a long time and will replace the image's existing SfM binaries.
RUN if [[ "${IKALIBR_REBUILD_SFM_CUDA}" == "1" ]]; then \
            export COLMAP_GIT_REF="${COLMAP_GIT_REF}"; \
            export GLOMAP_GIT_REF="${GLOMAP_GIT_REF}"; \
            export POSELIB_GIT_REF="${POSELIB_GIT_REF}"; \
            export COLMAP_CUDA_ARCHITECTURES="${COLMAP_CUDA_ARCHITECTURES}"; \
            export IKALIBR_BUILD_GLOMAP="${IKALIBR_BUILD_GLOMAP}"; \
            bash /tmp/install_sfm_cuda.sh; \
        fi

# Bake the iKalibr workspace source into .bashrc so every bash session
# (interactive shell, docker exec, non-login shells) works without manual sourcing.
RUN printf '\n# ── iKalibr workspace ─────────────────────────\n' >> /root/.bashrc \
        && echo 'source /home/iKalibr/install/setup.bash'             >> /root/.bashrc

RUN set -eux; \
    existing_group="$(getent group ${LOCAL_GID} | cut -d: -f1 || true)"; \
    if [ -n "${existing_group}" ] && [ "${existing_group}" != "fruc-docker" ]; then \
        groupmod -n fruc-docker "${existing_group}"; \
    elif ! getent group fruc-docker >/dev/null 2>&1; then \
        groupadd -o --gid ${LOCAL_GID} fruc-docker; \
    fi; \
    existing_user="$(getent passwd ${LOCAL_UID} | cut -d: -f1 || true)"; \
    if [ -n "${existing_user}" ] && [ "${existing_user}" != "fruc-docker" ]; then \
        old_home="$(getent passwd "${existing_user}" | cut -d: -f6)"; \
        usermod -l fruc-docker -g ${LOCAL_GID} -s /bin/bash "${existing_user}"; \
        if [ -n "${old_home}" ] && [ "${old_home}" != "/home/fruc-docker" ] && [ -d "${old_home}" ]; then \
            usermod -d /home/fruc-docker -m fruc-docker; \
        else \
            usermod -d /home/fruc-docker fruc-docker; \
            mkdir -p /home/fruc-docker; \
        fi; \
    elif ! id -u fruc-docker >/dev/null 2>&1; then \
        useradd -o -m -u ${LOCAL_UID} -g ${LOCAL_GID} -s /bin/bash fruc-docker; \
    else \
        usermod -o --uid ${LOCAL_UID} --gid ${LOCAL_GID} --home /home/fruc-docker fruc-docker; \
        mkdir -p /home/fruc-docker; \
    fi; \
    chown -R ${LOCAL_UID}:${LOCAL_GID} /home/fruc-docker; \
    echo "fruc-docker ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers.d/fruc-docker; \
    chmod 440 /etc/sudoers.d/fruc-docker

RUN cp /root/.bashrc /home/fruc-docker/.bashrc \
    && chown fruc-docker:fruc-docker /home/fruc-docker/.bashrc

RUN rm -f /tmp/install_sfm_cuda.sh

USER fruc-docker
ENV HOME=/home/fruc-docker
WORKDIR /home/fruc-docker
