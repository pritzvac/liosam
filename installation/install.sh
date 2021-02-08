#!/bin/bash

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

if [ -z $1 ]; then
  WORKSPACE_PATH=$GIT_PATH/workspace
else
  WORKSPACE_PATH="$1"
fi

# install dependencies
sudo apt update
sudo apt-get install libtbb-dev ros-melodic-pcl-* libpcl-* ros-melodic-navigation ros-melodic-robot-localization ros-melodic-robot-state-publisher ros-melodic-imu-complementary-filter

$SCRIPT_PATH/install_gtsam.sh
# $SCRIPT_PATH/install_pcl.sh

# cd $WORKSPACE_PATH
# catkin build
