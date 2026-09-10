#!/bin/bash
# Four-screen demo on the cluster: four random rallies tiled 2x2, streamed to
# a viser page through an SSH tunnel (and/or recorded to a video).
#   scripts/slurm_demo.sh [model.pt] [extra demo_quad.py args...]
#   scripts/slurm_demo.sh stop
# Defaults to the fine-tuned student. Examples:
#   scripts/slurm_demo.sh                                  # live page
#   scripts/slurm_demo.sh "" --record runs/demo.mp4 --no-viser   # 40 s video
# EXCLUDE=node1,node2 skips currently-broken GPU nodes; NODE=trpro-slurm1 pins one.
set -euo pipefail
PATH="/opt/slurm/bin:$PATH"
cd "$(dirname "$0")/.."

if [ "${1:-}" = "stop" ]; then
    scancel --name=quad-demo --user="$USER" && echo "demo job(s) cancelled"
    exit 0
fi
ckpt="${1:-}"
[ -n "$ckpt" ] || ckpt=$(ls -d logs/rsl_rl/badminton_student_ppo/2*/ | sort | tail -1)model_1499.pt
[ -f "$ckpt" ] || { echo "checkpoint not found: $ckpt" >&2; exit 1; }
shift || true
echo "checkpoint: $ckpt"

jid=$(sbatch --parsable --job-name=quad-demo --gres=shard:4096,tmpdisk:4096 \
    --exclude="${EXCLUDE:-tr-slurm2}" ${NODE:+-w "$NODE"} --cpus-per-task=4 --mem=16G --time=03:00:00 \
    --output=runs/demo-%j.out \
    --wrap="MUJOCO_GL=egl uv run scripts/demo_quad.py --checkpoint-file $ckpt $*")
echo "submitted job $jid; waiting for it to start..."
for _ in $(seq 1 120); do
    node=$(squeue -j "$jid" -h -o "%N" -t R 2>/dev/null || true)
    [ -n "$node" ] && break
    sleep 5
done
[ -n "${node:-}" ] || { echo "job $jid not running after 10 min (queued); check squeue" >&2; exit 1; }
echo "running on $node. Loads in ~1-2 min, then:"
echo "  ssh -L 8080:$node.cluster.watonomous.ca:8080 wato-login1"
echo "  open http://localhost:8080"
echo "log: runs/demo-$jid.out   stop with: scripts/slurm_demo.sh stop"
