# garment_fold_task-only image: the published isaac_lab image (stock Isaac Lab
# 2.3.2) plus the two fixes garment_fold_task needs that aren't safe to bake
# into the shared isaac_lab.Dockerfile without broader testing -- see
# src/simulation/garment_fold_task/README.md and PR #219 for why.
#
# This is a separate image/compose service (simulation_isaac_garment) so it
# touches nothing used by so101_vial_task / humanoid_rl* / humanoid_rl_tasks.
# Build with: ACTIVE_MODULES="simulation_isaac_garment" ./watod build
ARG SIMULATION_ISAAC_IMAGE=ghcr.io/watonomous/humanoid/simulation/isaac_lab
ARG TAG=dev_main
FROM ${SIMULATION_ISAAC_IMAGE}:${TAG}

ENV PYTHON=/workspace/isaaclab/_isaac_sim/python.sh

# 1. Isaac Lab 2.3.2 -> 2.3.0 downgrade. Stock 2.3.2's isaaclab/sim/views/
#    xform_prim_view.py has a Fabric-cache-backed pose read/write path that
#    serves stale poses to TiledCamera -- camera feeds visually freeze after
#    the first few frames and never update again, confirmed with a
#    continuously-driven action + frame-diffing test (not fixable by this
#    task's own `cfg.sim.use_fabric = False`, which is already set and does
#    not help -- the bug is inside isaaclab's own Fabric plumbing).
#    The lehome-official/IsaacLab fork (pinned to the LeHome Challenge's
#    validated 2.3.0) deletes the Fabric-backed methods outright, forcing
#    pose reads/writes through USD under the same public API -- no caller
#    needs to change.
RUN git clone --depth 1 https://github.com/lehome-official/IsaacLab.git /tmp/isaaclab_2.3.0 && \
    rm -rf /workspace/isaaclab/source/isaaclab/isaaclab && \
    cp -r /tmp/isaaclab_2.3.0/source/isaaclab/isaaclab /workspace/isaaclab/source/isaaclab/isaaclab && \
    rm -f /workspace/isaaclab/source/isaaclab/isaaclab/utils/warp/fabric.py && \
    rm -rf /tmp/isaaclab_2.3.0

# 2. open3d + libusb1.0: humanoid_garment_fold.utils.success_checker_garment's
#    primary particle-point read path (GarmentObject.get_current_mesh_points)
#    needs open3d, which isn't a dependency of this image or the garment
#    package; without it every success check throws and falls back to a
#    second path that also fails (deprecated ClothPrim API), so the checker
#    never fires at all.
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends libusb-1.0-0 && \
    rm -rf /var/lib/apt/lists/* && \
    $PYTHON -m pip install open3d
