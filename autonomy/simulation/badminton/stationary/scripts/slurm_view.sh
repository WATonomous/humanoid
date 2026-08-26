#!/bin/bash
# Start (or stop) a viser viewer session on the cluster for a checkpoint.
#   scripts/slurm_view.sh [model.pt]   -> newest checkpoint if omitted
#   scripts/slurm_view.sh stop
# Prints the ssh tunnel command once the job is running; the viewer then
# serves on http://localhost:8080 through that tunnel. The job holds a GPU
# shard for up to 3 h; stop it when done.
set -euo pipefail
PATH="/opt/slurm/bin:$PATH"
cd "$(dirname "$0")/.."

if [ "${1:-}" = "stop" ]; then
    scancel --name=viser-view --user="$USER" && echo "viewer job(s) cancelled"
    exit 0
fi

ckpt="${1:-$(ls -v "$(ls -td wandb/run-*/ | head -1)"files/model_*.pt | tail -1)}"
[ -f "$ckpt" ] || { echo "checkpoint not found: $ckpt" >&2; exit 1; }
echo "checkpoint: $ckpt"

jid=$(sbatch --parsable --job-name=viser-view --gres=shard:4096 --exclude=tr-slurm2 \
    --cpus-per-task=4 --mem=16G --time=03:00:00 --output=runs/viser-%j.out \
    --wrap="uv run scripts/play_rl.py Mjlab-Badminton-Receive-Teacher --viewer viser --device cuda:0 --num-envs 1 --checkpoint-file $ckpt")
echo "submitted job $jid; waiting for it to start..."
for _ in $(seq 1 120); do
    node=$(squeue -j "$jid" -h -o "%N" -t R 2>/dev/null || true)
    [ -n "$node" ] && break
    sleep 5
done
[ -n "${node:-}" ] || { echo "job $jid not running after 10 min (queued); check squeue" >&2; exit 1; }
echo "running on $node. Viewer needs ~3-5 min to load, then:"
echo "  ssh -L 8080:$node.cluster.watonomous.ca:8080 wato-login1"
echo "  open http://localhost:8080"
echo "stop with: scripts/slurm_view.sh stop"
