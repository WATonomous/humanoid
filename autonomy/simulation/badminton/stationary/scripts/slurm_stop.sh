#!/bin/bash
# Gracefully stop a training job launched by slurm_train.sbatch:
#   scripts/slurm_stop.sh <jobid>
# Signals only the batch shell (not the whole process group), so python
# gets SIGINT while wandb's sync process survives to upload and mark the
# run finished. Falls back to plain scancel if the job hangs around.
set -euo pipefail
PATH="/opt/slurm/bin:$PATH"
jid="$1"
scancel --batch --signal=USR1 "$jid"
echo "sent USR1 to batch shell of $jid; waiting for clean exit..."
for _ in $(seq 1 24); do
    sleep 5
    if ! squeue -j "$jid" -h 2>/dev/null | grep -q .; then
        echo "job $jid ended cleanly"
        exit 0
    fi
done
echo "job $jid still running after 120 s; forcing scancel"
scancel "$jid"
