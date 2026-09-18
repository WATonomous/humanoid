# pi0.5 (openpi) fine-tuning/inference container. Separate from isaac_lab.Dockerfile
# because openpi pins its own lerobot/jax/torch versions that conflict with it.
# Follows openpi's official Docker recipe (uv-based; plain pip can't resolve its deps).
#
# Launch: ACTIVE_MODULES="simulation_isaac_pi" ./watod up -d && ./watod -t simulation_isaac_pi_dev

FROM nvidia/cuda:12.2.2-cudnn8-runtime-ubuntu22.04@sha256:2d913b09e6be8387e1a10976933642c73c840c0b735f0bf3c28d97fc9bc422e0

ENV DEBIAN_FRONTEND=noninteractive
ENV HUMANOID_ROOT=/workspace/humanoid
ENV UV_LINK_MODE=copy
# venv lives outside /workspace/openpi so a bind mount can't shadow it
ENV UV_PROJECT_ENVIRONMENT=/.venv

# git-lfs: needed by LeRobot (openpi dependency)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git git-lfs linux-headers-generic build-essential clang ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

# uv via install script, not ghcr.io image (WATO's ghcr login denies other orgs' images)
RUN curl -LsSf https://astral.sh/uv/0.5.1/install.sh | env UV_INSTALL_DIR=/bin sh

WORKDIR /workspace

# Pinned commit; bump deliberately
ENV OPENPI_COMMIT=215abfb217dbac7d5f1273282331b9b1866c0479
RUN git clone https://github.com/Physical-Intelligence/openpi.git && \
    cd openpi && git checkout ${OPENPI_COMMIT} && \
    git submodule update --init --recursive

# Exact versions from openpi's uv.lock
WORKDIR /workspace/openpi
RUN uv venv --python 3.11.9 $UV_PROJECT_ENVIRONMENT
RUN GIT_LFS_SKIP_SMUDGE=1 uv sync --frozen --no-dev

# Patch installed `transformers` (required by pi0/pi0.5 model code)
RUN /.venv/bin/python -c "import transformers; print(transformers.__file__)" \
    | xargs dirname | xargs -I{} cp -r src/openpi/models_pytorch/transformers_replace/* {}

RUN mkdir -p /tmp/pycache && chmod 1777 /tmp/pycache
ENV PYTHONPYCACHEPREFIX=/tmp/pycache

COPY docker/simulation/isaac_lab/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN cat >> /root/.bashrc <<'EOF'
export HUMANOID_ROOT=/workspace/humanoid
# checkpoint cache (gs://openpi-assets downloads)
export OPENPI_DATA_HOME=${OPENPI_DATA_HOME:-/root/.cache/openpi}
alias pi05-train='cd /workspace/openpi && uv run scripts/train.py'
alias pi05-serve='cd /workspace/openpi && uv run scripts/serve_policy.py'
# run once per dataset before fine-tuning
alias pi05-compute-norm-stats='cd /workspace/openpi && uv run scripts/compute_norm_stats.py'
EOF

WORKDIR ${HUMANOID_ROOT}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
