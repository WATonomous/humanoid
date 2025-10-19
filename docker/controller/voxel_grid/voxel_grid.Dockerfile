FROM nvcr.io/nvidia/cuda:12.2.2-devel-ubuntu22.04

WORKDIR ${AMENT_WS}/src

COPY autonomy/controller/voxel_grid voxel_grid

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
      lsb-release && \
    rm -rf /var/lib/apt/lists/*

ARG INSTALL_ROS2=true
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    echo "Installing ROS 2 Humble" && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      | tee /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-ros-base && \
    rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc ; \
  fi

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install --no-cache-dir \
      pccm>=0.4.16 \
      ccimport>=0.4.4 \
      pybind11>=2.6.0 \
      numpy \
      fire

# Install prebuilt spconv-cu120 (compatible with CUDA 12.2 due to minor version compatibility)
RUN python3 -m pip install --no-cache-dir spconv-cu120

ENTRYPOINT ["./wato_ros_entrypoint.sh"]
