#!/bin/bash

WORKSPACE_PATH=$GIT_PATH/workspace

# install dependencies
sudo apt update
sudo apt-get install libtbb-dev ros-melodic-pcl-* libpcl-* ros-melodic-navigation ros-melodic-robot-localization ros-melodic-robot-state-publisher ros-melodic-imu-complementary-filter

./install_gtsam.sh
./install_pcl.sh

cd $WORKSPACE_PATH
catkin build
