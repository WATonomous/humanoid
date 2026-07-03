# Use the official, lightweight ROS 2 Humble image instead of a massive NVIDIA base
ARG BASE_IMAGE=ros:humble-ros-base-jammy

################################ Source ################################
FROM ${BASE_IMAGE} AS source

# Set up the workspace correctly
ENV AMENT_WS=/root/ament_ws
WORKDIR ${AMENT_WS}/src

# Copy in the required WATO source code (matching the Isaac architecture)
COPY autonomy/wato_msgs/sample_msgs sample_msgs
COPY autonomy/wato_msgs/common_msgs common_msgs

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | { grep 'apt-get install' || true; } \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list || true && \
    touch /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

ENV AMENT_WS=/root/ament_ws

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update -qq && \
    apt-get install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list) || true

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

ENV AMENT_WS=/root/ament_ws

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.sh" ]; then \
        . /opt/ros/$ROS_DISTRO/setup.sh && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL:-/opt/watonomous}; \
    fi

# Source and Build Artifact Cleanup
RUN rm -rf src/* build/* devel/* install/* log/*

# ── MjLab Physics & Web Viewer ────────────────────────────────────────────────
# No symlinks or custom python binaries needed. Just standard system pip.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir mujoco mjviser numpy jax[cuda12] brax flax optax

# ── ROS Networking & Entrypoint ──────────────────────────────────────────────
ENV ROS_DOMAIN_ID=0
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Append setup sourcing to bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'cd /root/ament_ws && colcon build --packages-select common_msgs 2>/dev/null || true' >> /root/.bashrc && \
    echo 'source /root/ament_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc

# Hook into the team's shared entrypoint script
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
RUN chmod +x ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]