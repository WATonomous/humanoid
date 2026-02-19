ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy source code
COPY autonomy/interfacing/can can
COPY autonomy/wato_msgs/sample_msgs sample_msgs

# Install rosdep if not present, update package lists
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# Update rosdep database (safe in containers)
RUN rosdep update

# Generate dependency list (simulated install → extract apt packages)
RUN rosdep install \
    --from-paths . \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    -y \
    --simulate | \
    grep "apt-get install" | \
    sed 's/apt-get install -y //' > /tmp/colcon_install_list || true


################################ Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Copy dependency list from source stage
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list

# Install dependencies + tools (update must be in same layer)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        $(cat /tmp/colcon_install_list) \
        can-utils \
        net-tools \
        iproute2 && \
    rm -rf /var/lib/apt/lists/*

# Copy source code into workspace
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src


################################ Build ################################
FROM dependencies AS build

WORKDIR ${AMENT_WS}

# Build ROS2 workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --install-base ${WATONOMOUS_INSTALL}

# Remove source + build artifacts (keeps only install)
RUN rm -rf src build log

# Pass udev symlinks into container
ENV UDEV=1

# Entrypoint
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

