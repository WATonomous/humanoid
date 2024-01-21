# After building a ROS2 workspace certain environment variables need to
# be set to find packages and load libraries. This is accomplished by sourcing
# the generated setup file. To avoid manually running this command everytime a
# container is brought up, this command is automatically executed when the
# .bashrc is sourced.
source /root/ament_ws/install/setup.bash

alias clw="rm -rf build/ && rm -rf install/"
alias dbd="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && source install/setup.bash"