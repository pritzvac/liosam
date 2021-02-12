#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

if [[ $# -eq 1 ]]
then
  WORKSPACE_PATH=$1
else
  WORKSPACE_PATH=$GIT_PATH/workspace
fi

# install dependencies
sudo apt update
sudo apt-get install libtbb-dev ros-melodic-pcl-* libpcl-* ros-melodic-navigation ros-melodic-robot-localization ros-melodic-robot-state-publisher ros-melodic-imu-complementary-filter

$SCRIPT_PATH/install_gtsam.sh $WORKSPACE_PATH
# $SCRIPT_PATH/install_pcl.sh

# cd $WORKSPACE_PATH
# catkin build
