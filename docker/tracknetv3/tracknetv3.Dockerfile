ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04
ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cu121

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

COPY autonomy/tracknetv3 tracknetv3

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update

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
ARG TORCH_INDEX_URL

COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list

RUN apt-get update && \
    apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

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
      libgl1-mesa-glx \
      libglib2.0-0 \
      libgtk-3-dev && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install --no-cache-dir \
      --index-url ${TORCH_INDEX_URL} \
      torch \
      torchvision

RUN python3 -m pip install --no-cache-dir \
      numpy \
      opencv-python \
      Pillow

WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build
COPY --from=source ${AMENT_WS}/src ${AMENT_WS}/src

WORKDIR ${AMENT_WS}

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --install-base ${WATONOMOUS_INSTALL}

RUN rm -rf build/* devel/* install/* log/*

COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
CMD ["/bin/bash"]
