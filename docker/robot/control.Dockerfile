# ================= Dependencies ===================
FROM ros:humble AS base

# ADD DEPENDENCIES HERE

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /root/ament_ws/src

COPY src/robot/control control
COPY src/wato_msgs/sample_msgs sample_msgs

WORKDIR /root/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /root/wato_ros_entrypoint.sh
COPY docker/.bashrc /root/.bashrc
ENTRYPOINT ["/root/wato_ros_entrypoint.sh"]
CMD ["ros2", "launch", "control", "control.launch.py"]
