#!/bin/bash
# First smoke run of the mjlab badminton teacher on the WATcloud SLURM cluster
# (https://cloud.watonomous.ca/docs/compute-cluster/slurm).
#
# From a login node (wato-login1 / wato-login2), with the repo cloned in $HOME:
#   cd ~/humanoid/autonomy/simulation/badminton/stationary
#   sbatch scripts/slurm_train.sbatch
# Tail progress:  tail -f runs/slurm-<jobid>.out
# Viewer is not started here; for play/viser use an interactive srun and an
# SSH tunnel to port 8080 (see README "training" section).
#
#SBATCH --job-name=badminton-rl
#SBATCH --partition=compute
#SBATCH --gres=gpu:1,tmpdisk:20480
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=08:00:00
#SBATCH --output=runs/slurm-%j.out

set -euo pipefail
cd "${SLURM_SUBMIT_DIR}"
mkdir -p runs

# uv is not installed cluster-wide; bootstrap into ~/.local/bin once.
if ! command -v uv >/dev/null 2>&1; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi
export PATH="$HOME/.local/bin:$PATH"

nvidia-smi
uv python install 3.12
uv sync --extra train

# Smoke-run scale: 1024 envs fits the shared-GPU VRAM floor; raise
# num-envs (README uses 4096) once this completes on a full GPU.
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Teacher \
    --env.scene.num-envs "${NUM_ENVS:-1024}"
