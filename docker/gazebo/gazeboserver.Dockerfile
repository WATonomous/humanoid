FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6 -y

RUN apt install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get -y update
RUN apt-get -y install ros-${ROS_DISTRO}-ros-gz ignition-fortress
RUN apt-get -y install ros-humble-velodyne-gazebo-plugins
RUN echo $GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib


# Add a docker user so that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \                                                                     
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \                                                                                                            
    chown root:root /usr/local/bin/fixuid && \                                                                              
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \                                                                                               
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml

USER docker:docker

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

FROM base as repo

USER docker:docker
WORKDIR /home/docker

ENV DEBIAN_FRONTEND interactive

COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/ros_entrypoint.sh /home/docker/ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc

ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/ros_entrypoint.sh"]

# CMD ["sleep", "infinity"]
# CMD ["ign", "launch", "launch/sim.ign"]

CMD ["ros2", "launch", "launch/sim.launch.py"]
