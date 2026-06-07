ARG BASE_IMAGE=nvcr.io/nvidia/isaac-sim:4.5.0

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY autonomy/samples/cpp/aggregator aggregator
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

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list) || true

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.sh" ]; then \
        . /opt/ros/$ROS_DISTRO/setup.sh && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL:-/opt/watonomous}; \
    fi

# Source and Build ArtifacFASTDDS_BUILTIN_TRANSPORTSt Cleanup
RUN rm -rf src/* build/* devel/* install/* log/*

# ── Isaac Lab 2.3.2 ───────────────────────────────────────────────────────────
# Isaac Sim is at /isaac-sim/ in the base image. Isaac Lab expects it at
# _isaac_sim/ relative to its own root, so we create a symlink.
RUN apt-get update && apt-get install -y --no-install-recommends git && \
    rm -rf /var/lib/apt/lists/* && \
    git clone --branch v2.3.2 --depth 1 \
        https://github.com/isaac-sim/IsaacLab.git /isaac-lab && \
    ln -s /isaac-sim /isaac-lab/_isaac_sim && \
    /isaac-lab/_isaac_sim/kit/python/bin/python3 -m pip install --upgrade pip setuptools && \
    /isaac-lab/_isaac_sim/kit/python/bin/python3 -m pip install -e /isaac-lab/source/isaaclab && \
    /isaac-lab/_isaac_sim/kit/python/bin/python3 -m pip install mujoco

ENV ISAACLAB_PATH=/isaac-lab

ENV ROS_DOMAIN_ID=0
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Install ROS2 Humble and wire up the shared entrypoint
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-ros-base python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p /opt/watonomous && echo 'source /opt/ros/humble/setup.bash' > /opt/watonomous/setup.bash

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'cd /root/ament_ws && colcon build --packages-select common_msgs quest_isaac_teleop 2>/dev/null || true' >> /root/.bashrc && \
    echo 'source /root/ament_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
