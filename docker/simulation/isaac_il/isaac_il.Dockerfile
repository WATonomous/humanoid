# SO101 sim IL: NVIDIA workshop stack (Isaac Lab 2.3.2 / Sim 5.1 / Python 3.11 / torch 2.7).
# Launch via watod: ACTIVE_MODULES="simulation_il" ./watod up -d && ./watod -t simulation_il_dev
FROM nvcr.io/nvidia/isaac-lab:2.3.2

ENV PYTHON=/workspace/isaaclab/_isaac_sim/python.sh
ENV PATH="/workspace/isaaclab/_isaac_sim/kit/python/bin:$PATH"
ENV ISAACLAB_PATH=/workspace/isaaclab
ENV HUMANOID_ROOT=/workspace/humanoid

# ── LeRobot (workshop coexist pattern: no-deps + constraints) ─────────────────
WORKDIR /workspace
RUN git clone https://github.com/huggingface/lerobot.git && \
    cd lerobot && git checkout e670ac5daf9b76

RUN $PYTHON -m pip install --no-deps -e /workspace/lerobot

RUN printf '%s\n' \
    "packaging==23.0" \
    "numpy==1.26.0" \
    "lxml==4.9.4" \
    "torch==2.7.0" \
    "torchvision==0.22.0" \
    "imageio==2.37.0" \
    > /tmp/constraints.txt

RUN $PYTHON -m pip install -c /tmp/constraints.txt \
    "datasets>=4.0.0,<4.2.0" \
    "diffusers>=0.27.2,<0.36.0" \
    "huggingface-hub[hf-transfer,cli]>=0.34.2,<0.36.0" \
    "accelerate>=1.10.0,<2.0.0" \
    "cmake>=3.29.0.1,<4.2.0" \
    "av>=15.0.0,<16.0.0" \
    "jsonlines>=4.0.0,<5.0.0" \
    "pynput>=1.7.7,<1.9.0" \
    "pyserial>=3.5,<4.0" \
    "wandb>=0.20.0,<0.22.0" \
    "torchcodec>=0.2.1,<0.6.0" \
    "draccus==0.10.0" \
    "deepdiff>=7.0.1,<9.0.0" \
    "feetech-servo-sdk>=1.0.0,<2.0.0"

RUN curl --proto "=https" --tlsv1.2 -sSf -L -o /tmp/ffmpeg.tar.xz \
    https://github.com/BtbN/FFmpeg-Builds/releases/download/latest/ffmpeg-n7.1-latest-linux64-lgpl-shared-7.1.tar.xz && \
    tar -xf /tmp/ffmpeg.tar.xz -C /usr/local --strip-components=1 && \
    ldconfig && \
    rm /tmp/ffmpeg.tar.xz

RUN apt-get update && apt-get install -y --no-install-recommends \
    libx11-6 libxcursor1 libxrandr2 libxi6 libxinerama1 \
    libxkbcommon0 libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

RUN $PYTHON -m pip install --no-deps "rerun-sdk>=0.24.0,<0.27.0" && \
    $PYTHON -m pip install pyzmq && \
    $PYTHON -m pip install --upgrade pip

# ── Humanoid IL packages (editable; repo bind-mounted at runtime) ─────────────
COPY autonomy/il ${HUMANOID_ROOT}/autonomy/il
COPY autonomy/simulation/so101_vial_task ${HUMANOID_ROOT}/autonomy/simulation/so101_vial_task

# Humanoid packages: --no-deps only (never [sim]/[lerobot] extras — they pull torch/lerobot with deps).
RUN $PYTHON -m pip install --no-deps -e "${HUMANOID_ROOT}/autonomy/il" && \
    $PYTHON -m pip install -c /tmp/constraints.txt psutil && \
    $PYTHON -m pip install --no-deps -e "${HUMANOID_ROOT}/autonomy/simulation/so101_vial_task"

RUN mkdir -p /tmp/pycache && chmod 1777 /tmp/pycache
ENV PYTHONPYCACHEPREFIX=/tmp/pycache

COPY docker/simulation/isaac_il/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN cat >> /root/.bashrc <<'EOF'
export ISAACLAB=/workspace/isaaclab
export HUMANOID_ROOT=/workspace/humanoid
export TASK_ROOT=/workspace/humanoid/autonomy/simulation/so101_vial_task
alias il-train='$PYTHON -m lerobot.scripts.lerobot_train'
alias il-record='cd $TASK_ROOT && PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_agent.py'
alias il-eval='cd $TASK_ROOT && PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p scripts/lerobot_eval.py'
EOF

WORKDIR ${HUMANOID_ROOT}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
