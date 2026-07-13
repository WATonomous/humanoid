# ── isaac_lab_il_datagen: Isaac Lab + cuRobo for pick_place_gen demo generation ──
# Thin fork of the shared simulation_isaac (isaac_lab) image that adds ONLY the
# cuRobo GPU motion-planning stack. This keeps cuRobo and its CUDA-dev headers
# out of the shared container — nothing but the IL data-generation pipeline
# needs them.
#
# BASE_IMAGE must be a clean isaac_lab image: ROS 2 + Isaac Sim 2.3.2 + LeRobot
# + humanoid_il, with NO cuRobo. It provides $PYTHON and /tmp/constraints.txt,
# which the cuRobo install below relies on. watod passes this (the shared
# SIMULATION_ISAAC_IMAGE:TAG) via the compose module; the default here is only a
# fallback for building the Dockerfile directly. Build or pull the base first.
ARG BASE_IMAGE=ghcr.io/watonomous/humanoid/simulation/isaac_lab:main
FROM ${BASE_IMAGE}

# ── cuRobo (GPU motion planning for pick_place_gen demo generation) ──────────
# New-API cuRobo JIT-compiles CUDA kernels at runtime via NVRTC (cuda-core
# backend) — no nvcc/build-time compile. It needs CUDA headers (cudart, nvrtc,
# cccl) matching torch's CUDA (12.8). Kernels JIT fine on Blackwell (sm_120).
RUN curl -sSL -o /tmp/cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-dev-12-8 cuda-cccl-12-8 cuda-nvrtc-dev-12-8 cuda-profiler-api-12-8 && \
    rm -rf /var/lib/apt/lists/*

# Pinned to the commit validated with the wato_bimanual_arm robot config.
# Constraints (inherited from the base image at /tmp/constraints.txt) protect
# the torch/numpy pins. Post-install fixups:
#  - websockets>=13 required by curobo's viser dep (repo code imports none;
#    verified nothing else pins 12.x)
#  - lxml restored to the image's shipped 5.4.0 (constraints would downgrade
#    it to 4.9.4, breaking dex-retargeting's >=5.2.2 requirement)
RUN git clone https://github.com/NVlabs/curobo.git /workspace/curobo && \
    cd /workspace/curobo && git checkout a35a708 && \
    $PYTHON -m pip install -e /workspace/curobo -c /tmp/constraints.txt && \
    $PYTHON -m pip install -c /tmp/constraints.txt "cuda-core[cu12]" && \
    $PYTHON -m pip install --no-deps websockets==16.0 lxml==5.4.0
