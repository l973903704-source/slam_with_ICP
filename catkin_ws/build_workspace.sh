#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
cd "$(dirname "$0")"
chmod +x src/slamware_loop_slam/scripts/*.py
catkin_make
source devel/setup.bash

echo "workspace build finished"
