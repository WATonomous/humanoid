# mjlab (MuJoCo Warp) GPU training container for the badminton receive env.
# Headless: training + viser web viewer on port 8080 (compose maps it).
# Requires the NVIDIA container toolkit; mjlab training needs an NVIDIA GPU.
ARG BASE_IMAGE=ros:humble-ros-base-jammy

################################ Source ################################
FROM ${BASE_IMAGE} AS source

ENV AMENT_WS=/root/ament_ws
WORKDIR ${AMENT_WS}/src

COPY autonomy/wato_msgs/common_msgs common_msgs

RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | { grep 'apt-get install' || true; } \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list || true && \
    touch /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

ENV AMENT_WS=/root/ament_ws

COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    apt-get install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list) || true

WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

ENV AMENT_WS=/root/ament_ws

WORKDIR ${AMENT_WS}
RUN if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.sh" ]; then \
        . /opt/ros/$ROS_DISTRO/setup.sh && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL:-/opt/watonomous}; \
    fi

RUN rm -rf src/* build/* devel/* install/* log/*

# ── Training stack: uv + Python 3.12 ─────────────────────────────────────────
# The badminton project (mounted at /root/ament_ws/src/simulation/badminton/
# stationary) manages its own env with uv; the container just supplies uv,
# an interpreter, and libs mujoco needs for headless EGL rendering.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl git libegl1 libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"
RUN uv python install 3.12

# Pre-warm the training venv so `watod` users don't pay the install at
# runtime; the mounted project reuses this cache (same uv cache dir).
ENV UV_LINK_MODE=copy

# ── ROS Networking & Entrypoint ──────────────────────────────────────────────
ENV ROS_DOMAIN_ID=0
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'cd /root/ament_ws && colcon build --packages-select common_msgs 2>/dev/null || true' >> /root/.bashrc && \
    echo 'source /root/ament_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc && \
    echo '# badminton mjlab training: cd into the project and sync the env' >> /root/.bashrc && \
    echo 'alias badminton="cd /root/ament_ws/src/simulation/badminton/stationary && uv sync --extra train"' >> /root/.bashrc

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
RUN chmod +x ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
