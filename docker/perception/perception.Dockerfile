ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/perception/depth_estimation ${AMENT_WS}/src/perception/depth_estimation
COPY src/wato_msgs/sample_msgs ${AMENT_WS}/src/wato_msgs/sample_msgs

# Check if src directory is correctly copied
RUN ls -al ${AMENT_WS}/src

# Update rosdep
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && (rosdep init || echo "rosdep already initialized, skipping...") \
    && rosdep update

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libssl-dev \
    usbutils \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone realsense-ros r/4.56.3 (wrapper)
RUN git clone --branch r/4.56.3 --depth=1 https://github.com/IntelRealSense/realsense-ros.git ${AMENT_WS}/src/realsense-ros

# Download and extract matching librealsense SDK (v2.56.3)
RUN cd ${AMENT_WS}/src/realsense-ros && \
    wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.56.3.tar.gz && \
    tar xvf v2.56.3.tar.gz && \
    mv librealsense-2.56.3 librealsense && \
    rm v2.56.3.tar.gz

# Build librealsense SDK first
RUN cd ${AMENT_WS}/src/realsense-ros/librealsense && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
             -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false && \
    make -j$(nproc) && make install

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src
COPY --from=source /usr/local/lib /usr/local/lib
COPY --from=source /usr/local/include /usr/local/include

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    libssl3 \
    && ldconfig \
    && rm -rf /var/lib/apt/lists/*

# Install Rosdep requirements
RUN apt-get update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y \
    && rm -rf /var/lib/apt/lists/*

# Dependency Cleanup
RUN apt-get autoremove -y && apt-get clean && \
    rm -rf /root/.cache /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --install-base ${WATONOMOUS_INSTALL} \
    && rm -rf build log

# Entrypoint will run before any CMD on launch
COPY docker/wato_ros_entrypoint.sh /wato_ros_entrypoint.sh
ENTRYPOINT ["/wato_ros_entrypoint.sh"]