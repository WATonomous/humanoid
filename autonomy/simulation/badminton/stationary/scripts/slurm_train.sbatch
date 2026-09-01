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
# Any CUDA GPU the cu128 torch wheels support (sm_75+: rtx_2080_ti on
# delta-slurm1, rtx_3090 x4 on trpro-slurm1, rtx_4090 on trpro-slurm2);
# tr-slurm2's gtx_1080 (sm_61) is excluded ("no kernel image", job 593623).
# A shard (MiB of VRAM) rather than the exclusive gpu:1: training uses
# < 1 GB, and an exclusive request queues for hours behind anyone holding
# a shard. Sized to fit the gaps on a busy cluster: 1024 envs measured
# 854 MiB VRAM and the loop is GPU-bound, so 4 GB / 4 CPUs / 16 GB RAM is
# ample. First run on a new GPU arch recompiles the Warp kernels (~5 min).
#SBATCH --gres=shard:4096,tmpdisk:20480
#SBATCH --exclude=tr-slurm2
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
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
# Graceful stop path: use scripts/slurm_stop.sh <jobid>, which signals ONLY
# this batch shell (scancel --batch --signal=USR1). Plain scancel SIGTERMs
# every process at once, killing wandb's sync process before it can upload,
# so the run shows as crashed no matter what python does. The USR1 trap
# SIGINTs just the python process; its wandb service child stays alive to
# sync, and train_rl.py finishes the run on KeyboardInterrupt.
uv run scripts/train_rl.py Mjlab-Badminton-Receive-Teacher \
    --env.scene.num-envs "${NUM_ENVS:-1024}" ${EXTRA_ARGS:-} &
TRAIN_PID=$!
# uv run does not reliably forward SIGINT to its python child: signal the
# children first, then uv itself
trap 'pkill -INT -P "$TRAIN_PID"; kill -INT "$TRAIN_PID"' USR1 TERM INT
wait "$TRAIN_PID" || wait "$TRAIN_PID"
