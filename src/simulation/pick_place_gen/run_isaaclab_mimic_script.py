"""Thin wrapper to run IsaacLab Mimic's stock scripts (annotate_demos.py,
generate_dataset.py) with the custom pick_place_bimanual task registered.

Confirmed by reading their source: those stock scripts only import IsaacLab's
own built-in task packages (isaaclab_tasks, isaaclab_mimic.envs) at import
time -- there is no external-task discovery hook. Run directly, gym.spec()
never finds Isaac-PickPlace-BimanualLeft-Mimic-v0.

This wrapper runs the target script's module-level code (arg parsing,
AppLauncher, its own task imports) via runpy, using a run_name other than
"__main__" so its own `if __name__ == "__main__":` block does NOT fire.
Once that returns (AppLauncher/simulation_app now exist, same as if the
target script had been run directly up to that guard), it imports our task
package -- registering Isaac-PickPlace-BimanualLeft-Mimic-v0 into the same
process-global gym registry the target script's later gym.spec() call reads
from -- then calls the target's own main() and closes the sim app itself,
mirroring exactly what that guarded block would have done.

NOTE: run these Mimic tooling steps WITHOUT --enable_cameras. The datagen
Mimic env is state-only (its observation group is joint/object state, no
image terms; source demos are recorded cameras-off), and
PickPlaceBimanualMimicEnvCfg.__post_init__ drops the scene cameras for
exactly this reason. Passing --enable_cameras spins up the RTX render
pipeline unnecessarily, whose worker-thread init deadlocked reproducibly
during headless scene setup on this setup.

Usage: identical to the target script, with the target script's path
prepended as the first argument:
    isaaclab.sh -p run_isaaclab_mimic_script.py <target_script.py> --task ... [rest of target's own flags]
"""
import runpy
import sys
import traceback
from pathlib import Path

target_script = sys.argv[1]
sys.argv = [sys.argv[0]] + sys.argv[2:]  # target script only ever sees its own expected argv

_GEN_DIR = Path(__file__).resolve().parent
_HRL_DIR = _GEN_DIR.parents[1] / "Humanoid_Wato" / "HumanoidRL"
for _p in (str(_GEN_DIR), str(_HRL_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

namespace = runpy.run_path(target_script, run_name="isaaclab_mimic_wrapper")

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: E402,F401  (registers Isaac-PickPlace-BimanualLeft-Mimic-v0)

try:
    result = namespace["main"]()
except BaseException:
    # Print the traceback HERE, before the finally's simulation_app.close().
    # Kit's shutdown can hang, and if it does inside finally the exception
    # never propagates to the interpreter's default handler -- so without
    # this the real error is silently swallowed by the shutdown hang.
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    raise
finally:
    # Neither annotate_demos.py nor generate_dataset.py wrap main() in
    # try/finally themselves (checked: their own __main__ guards call
    # main() then close() unconditionally on the next line), so on an
    # uncaught exception simulation_app.close() never runs and Kit's
    # non-daemon background threads keep the process alive indefinitely.
    # Always closing here, even on failure, avoids that hang.
    namespace["simulation_app"].close()

if result is not None:
    sys.exit(result)
