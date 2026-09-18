# ── pi0.5 (openpi) inference/fine-tuning container ──────────────────────────
# Mirrors openpi's own official Docker recipe (scripts/docker/serve_policy.Dockerfile
# in the openpi repo) rather than a hand-rolled pip install: openpi manages deps
# via uv + an internal workspace member (packages/openpi-client) that plain pip
# can't resolve — openpi-client isn't published to PyPI, and openpi pins its own
# lerobot commit via a git rev in [tool.uv.sources], not the PyPI "lerobot" package.
#
# Kept as its own module (not merged into isaac_lab.Dockerfile) because openpi
# pins a *different* lerobot commit + its own jax/torch versions that would
# conflict with the isaac_lab image's separately-pinned lerobot commit and
# torch build. This container talks to the sim side over openpi's built-in
# remote-inference websocket client/server (see docs/remote_inference.md in
# the openpi repo) instead of sharing a Python env.
#
# Launch via watod: ACTIVE_MODULES="simulation_isaac_pi" ./watod up -d && ./watod -t simulation_isaac_pi_dev

FROM nvidia/cuda:12.2.2-cudnn8-runtime-ubuntu22.04@sha256:2d913b09e6be8387e1a10976933642c73c840c0b735f0bf3c28d97fc9bc422e0

ENV DEBIAN_FRONTEND=noninteractive
ENV HUMANOID_ROOT=/workspace/humanoid
# Copy from uv's cache instead of hardlinking (needed across the mounted-volume boundary).
ENV UV_LINK_MODE=copy
# Keep the venv outside /workspace/openpi so it doesn't get shadowed if that dir is ever bind-mounted.
ENV UV_PROJECT_ENVIRONMENT=/.venv

# git-lfs is needed because LeRobot (an openpi dependency) uses it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git git-lfs linux-headers-generic build-essential clang ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

# Installed via the official script instead of `COPY --from=ghcr.io/astral-sh/uv` —
# avoids ghcr.io entirely (this repo's Docker login for the watonomous org denies
# pulls of *any* other org's images from ghcr, including public ones like this).
RUN curl -LsSf https://astral.sh/uv/0.5.1/install.sh | env UV_INSTALL_DIR=/bin sh

WORKDIR /workspace

# Pinned to a commit we've verified builds. Bump deliberately — never track `main`.
ENV OPENPI_COMMIT=215abfb217dbac7d5f1273282331b9b1866c0479
RUN git clone https://github.com/Physical-Intelligence/openpi.git && \
    cd openpi && git checkout ${OPENPI_COMMIT} && \
    git submodule update --init --recursive

# Installs the exact versions pinned in openpi's own uv.lock — no guessed deps.
WORKDIR /workspace/openpi
RUN uv venv --python 3.11.9 $UV_PROJECT_ENVIRONMENT
RUN GIT_LFS_SKIP_SMUDGE=1 uv sync --frozen --no-dev

# openpi patches a few files inside the installed `transformers` package
# (required for the pi0/pi0.5 model code to import) — same step as openpi's
# own serve_policy.Dockerfile.
RUN /.venv/bin/python -c "import transformers; print(transformers.__file__)" \
    | xargs dirname | xargs -I{} cp -r src/openpi/models_pytorch/transformers_replace/* {}

# ── wato_bimanual_arm pick_place fine-tuning config ─────────────────────────
# openpi's own docs say to "copy this class and modify" per-robot config classes
# (see LeRobotLiberoDataConfig in their config.py) rather than expose a plugin
# hook — there isn't one. We vendor the diff as two files kept in our own repo
# (reviewable, survives `openpi_commit` bumps as a visible merge conflict
# instead of a silent break) and apply them at build time:
#   - pick_place_policy.py: COPY'd in as a new file under openpi's policies/.
#   - register_pickplace_config.py: appended to the end of config.py. Safe to
#     append rather than insert mid-file because _CONFIGS_DICT (built earlier
#     in that file from the _CONFIGS list) is just a plain module-level dict —
#     mutating it after the fact from appended code has the same effect as if
#     our entry had been in the original list.
# UNVERIFIED — see pick_place_policy.py's docstring for what to check before
# trusting this config for a real fine-tuning run.
COPY src/pi05/pick_place_policy.py src/openpi/policies/pick_place_policy.py
COPY src/pi05/register_pickplace_config.py /tmp/register_pickplace_config.py
RUN cat /tmp/register_pickplace_config.py >> src/openpi/training/config.py

# TODO: once a websocket-client wrapper exists (using openpi's own
# remote-inference client — see docs/remote_inference.md in the openpi repo —
# to talk to this container from simulation_isaac), add it under src/pi05/ too.

RUN mkdir -p /tmp/pycache && chmod 1777 /tmp/pycache
ENV PYTHONPYCACHEPREFIX=/tmp/pycache

COPY docker/simulation/isaac_lab/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN cat >> /root/.bashrc <<'EOF'
export HUMANOID_ROOT=/workspace/humanoid
# Where openpi caches/looks for downloaded base checkpoints (gs://openpi-assets/...).
export OPENPI_DATA_HOME=${OPENPI_DATA_HOME:-/root/.cache/openpi}
alias pi05-train='cd /workspace/openpi && uv run scripts/train.py'
alias pi05-serve='cd /workspace/openpi && uv run scripts/serve_policy.py'
# Run once per dataset before fine-tuning, e.g.: pi05-compute-norm-stats --config-name pi05_pickplace_bimanual
alias pi05-compute-norm-stats='cd /workspace/openpi && uv run scripts/compute_norm_stats.py'
EOF

WORKDIR ${HUMANOID_ROOT}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
