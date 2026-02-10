ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

ARG AMENT_WS=/root/ament_ws
WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY autonomy/wato_msgs/common_msgs wato_msgs/common_msgs

RUN git clone https://github.com/OctoMap/octomap_mapping.git

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
      python3 \
      python3-pip \
      python3-dev \
      python3-setuptools \
      ros-humble-octomap \
      ros-humble-octomap-msgs \
      ros-humble-octomap-ros \
      ros-humble-octomap-rviz-plugins \
      ros-humble-pcl-ros \
      ros-humble-pcl-conversions \
      ros-humble-image-transport \
      ros-humble-depth-image-proc \
      ros-humble-tf2-ros \
      ros-humble-tf2-geometry-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install --no-cache-dir \
      pccm>=0.4.16 \
      ccimport>=0.4.4 \
      pybind11>=2.6.0 \
      numpy \
      fire 

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
ARG AMENT_WS=/root/ament_ws

# Build ROS2 packages
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Build Artifact Cleanup (keep src for development)
RUN rm -rf build/* devel/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
