ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04
ARG AMENT_WS=/root/ament_ws

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY autonomy/perception perception
COPY autonomy/wato_msgs/common_msgs wato_msgs/common_msgs

# Scan for rosdeps
# RUN apt-get -qq update
# RUN rosdep update
# RUN rosdep install --from-paths . --ignore-src -r -s \
#         | grep 'apt-get install' \
#         | awk '{print $3}' \
#         | sort  > /tmp/colcon_install_list

RUN apt-get -qq update
RUN rosdep update
RUN echo "" > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

# Avoid interactive package prompts
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Essential build & Python tooling
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      git \
      cmake \
      ninja-build \
      python3 \
      python3-pip \
      python3-dev \
      python3-setuptools \
      curl \
      ca-certificates \
      gnupg2 \
      libgl1-mesa-glx \
      lsb-release \
      libssl-dev \
      usbutils \
      libusb-1.0-0-dev \
      pkg-config \
      libgtk-3-dev && \
      rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install --no-cache-dir \
      pccm>=0.4.16 \
      ccimport>=0.4.4 \
      pybind11>=2.6.0 \
      numpy \
      fire \
      cv_bridge \
      opencv-python


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
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DFORCE_RSUSB_BACKEND=ON \
             -DBUILD_EXAMPLES=false \
             -DBUILD_GRAPHICAL_EXAMPLES=false && \
    make -j$(nproc) && make install

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
COPY --from=dependencies ${AMENT_WS}/src/realsense-ros ${AMENT_WS}/src/realsense-ros
# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=false \
        -DBUILD_GRAPHICAL_EXAMPLES=false \
        --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
