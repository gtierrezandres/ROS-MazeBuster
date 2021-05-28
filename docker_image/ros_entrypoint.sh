#!/bin/bash
set -e
export DEBIAN_FRONTEND=noninteractive

# setup ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

unset DEBIAN_FRONTEND
exec "$@"
