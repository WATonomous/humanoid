ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

ARG AMENT_WS=/root/ament_ws
WORKDIR ${AMENT_WS}/src

COPY autonomy/behaviour/joint_command joint_command
COPY autonomy/wato_msgs/common_msgs common_msgs

RUN apt-get -qq update && rosdep update && echo "" > /tmp/colcon_install_list

################################ Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      libyaml-cpp-dev \
      $(cat /tmp/colcon_install_list) && \
    rm -rf /var/lib/apt/lists/*

WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

################################ Build ################################
FROM dependencies AS build

WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --install-base ${WATONOMOUS_INSTALL}

RUN rm -rf src build log

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
