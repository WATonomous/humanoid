# garment_fold_task-only image: published isaac_lab (stock 2.3.2) plus two
# fixes this task needs that aren't safe to bake into the shared
# isaac_lab.Dockerfile without broader testing (see garment_fold_task/README.md
# + PR #219). Separate image/service (simulation_isaac_garment) so it touches
# nothing used by so101_vial_task / humanoid_rl*.
# Build: ACTIVE_MODULES="simulation_isaac_garment" ./watod build
ARG SIMULATION_ISAAC_IMAGE=ghcr.io/watonomous/humanoid/simulation/isaac_lab
ARG TAG=dev_main
FROM ${SIMULATION_ISAAC_IMAGE}:${TAG}

ENV PYTHON=/workspace/isaaclab/_isaac_sim/python.sh

# 1. 2.3.2 -> 2.3.0: stock 2.3.2's xform_prim_view.py Fabric pose cache serves
#    stale poses to TiledCamera (feeds freeze after a few frames; confirmed by
#    frame-diffing a continuously-driven action -- `cfg.sim.use_fabric=False`
#    doesn't help, the bug is inside isaaclab itself). lehome-official/IsaacLab
#    (2.3.0) drops the Fabric path, forcing USD under the same public API.
RUN git clone --depth 1 https://github.com/lehome-official/IsaacLab.git /tmp/isaaclab_2.3.0 && \
    rm -rf /workspace/isaaclab/source/isaaclab/isaaclab && \
    cp -r /tmp/isaaclab_2.3.0/source/isaaclab/isaaclab /workspace/isaaclab/source/isaaclab/isaaclab && \
    rm -f /workspace/isaaclab/source/isaaclab/isaaclab/utils/warp/fabric.py && \
    rm -rf /tmp/isaaclab_2.3.0

# 2. open3d + libusb1.0: success_checker_garment's primary read path needs
#    open3d (not a base-image dep); without it every success check throws and
#    falls back to a second path that also fails (deprecated ClothPrim API).
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends libusb-1.0-0 && \
    rm -rf /var/lib/apt/lists/* && \
    $PYTHON -m pip install open3d
