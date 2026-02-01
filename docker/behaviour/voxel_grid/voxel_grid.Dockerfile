ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble

################################ Source ################################
FROM ${BASE_IMAGE} AS source

ARG AMENT_WS=/root/ament_ws
WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY autonomy/behaviour/voxel_grid voxel_grid
COPY autonomy/wato_msgs/common_msgs wato_msgs/common_msgs

# Scan for rosdeps
# RUN apt-get -qq update
# RUN rosdep update
# RUN rosdep install --from-paths . --ignore-src -r -s \
#         | grep 'apt-get install' \
#         | awk '{print $3}' \Is 
#         | sort  > /tmp/colcon_install_list

RUN apt-get -qq update
RUN rosdep update
RUN echo "" > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

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
      lsb-release && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install --no-cache-dir \
      pccm>=0.4.16 \
      ccimport>=0.4.4 \
      pybind11>=2.6.0 \
      numpy \
      fire \
      torch 

# Install prebuilt spconv-cu120 (compatible with CUDA 12.2 due to minor version compatibility)
RUN python3 -m pip install --no-cache-dir spconv-cu120

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
ARG AMENT_WS=/root/ament_ws

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
