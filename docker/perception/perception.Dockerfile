ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY autonomy/perception perception
COPY autonomy/wato_msgs/common_msgs wato_msgs/common_msgs

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

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list

RUN apt-get update && \
    apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

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
      libgtk-3-dev

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-$ROS_DISTRO-librealsense2* \
      ros-$ROS_DISTRO-realsense2-camera* && \
      rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install --no-cache-dir \
      pccm>=0.4.16 \
      ccimport>=0.4.4 \
      pybind11>=2.6.0 \
      cv_bridge \
      numpy \
      fire \
      opencv-python

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
COPY --from=source ${AMENT_WS}/src ${AMENT_WS}/src

# Build ROS2 packages
WORKDIR ${AMENT_WS}

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
