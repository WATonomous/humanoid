ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy source code
COPY src/interfacing/can can
COPY src/interfacing/dbc dbc
COPY src/common_msgs common_msgs

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

# Apt dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libboost-all-dev \
    libxml2-dev \
    can-utils \
    net-tools \
    iproute2 \
    python3-yaml \
    $(cat /tmp/colcon_install_list) \
    && rm -rf /var/lib/apt/lists/*

# # CAN dbc parser
WORKDIR /usr/local
RUN git clone --recurse-submodules https://github.com/xR3b0rn/dbcppp.git

# Build and install dbcppp
WORKDIR /usr/local/dbcppp
RUN mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_SHARED_LIBS=ON .. && \
    make -j && \
    make install && \
    ldconfig

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




################################ Develop ################################
# Run as the host user so bind-mounted files aren't root-owned. The base image
# ships a `bolty` user at uid 1000; remap it to the host user (or make a new one).
FROM build AS develop
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=dev
RUN old=$(getent passwd "${USER_UID}" | cut -d: -f1 || true); \
    if [ -n "$old" ] && [ "$old" != "${USERNAME}" ]; then \
        groupmod -n "${USERNAME}" "$(getent group "${USER_GID}" | cut -d: -f1)" 2>/dev/null || true; \
        usermod  -l "${USERNAME}" -d "/home/${USERNAME}" -m "$old"; \
    fi; \
    id -u "${USERNAME}" >/dev/null 2>&1 || { \
        getent group "${USER_GID}" >/dev/null || groupadd --gid "${USER_GID}" "${USERNAME}"; \
        useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}" --shell /bin/bash; }; \
    apt-get update && apt-get install -y --no-install-recommends sudo; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    chown -R "${USER_UID}:${USER_GID}" "${AMENT_WS}" "/home/${USERNAME}"; \
    rm -rf /var/lib/apt/lists/*
USER ${USERNAME}
WORKDIR ${AMENT_WS}
