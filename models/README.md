# `models/`

Frozen RL policy baselines worth keeping across the team — checked in so they
travel with the repo.

| dir | task / gym id | notes |
|-----|---------------|-------|
| `push/` | `Isaac-Bimanual-Push-Block-v0` | teacher policy (`model_1499.pt`) for the vision-distillation run |
| `push_distill/` | `Isaac-Bimanual-Push-Block-Distill-v0` | distilled vision student (`student_699.pt`) |
| `pioneer_hand_cube/` | `Isaac-Repose-Cube-PioneerHand-v0` | in-hand reorientation (`model_6600.pt`) |
| `pioneer_humanoid_flat/` | `Isaac-Locomotion-Flat-PioneerHumanoid-v0` | flat-ground walking (`model_3900.pt`) |
| `pioneer_humanoid_rough/` | `Isaac-Locomotion-Rough-PioneerHumanoid-v0` | rough-terrain walking (`model_2999.pt`) |

**Live training runs go to `outputs/rl/<experiment>/` (gitignored)** — `play.py`
with no `--checkpoint` loads the latest from there. Only promote a checkpoint
here when it's a baseline the team should share; keep it to one `.pt` per task.
