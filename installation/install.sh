#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
GIT_PATH="${HOME}/git"
WORKSPACE_PATH="${HOME}/workspace"

# install dependencies
# echo "deb http://cz.archive.ubuntu.com/ubuntu eoan main universe" | sudo tee -a  /etc/apt/sources.list
# sudo apt update
sudo apt install libtbb-dev ros-melodic-pcl-* libpcl-* ros-melodic-navigation ros-melodic-robot-localization ros-melodic-robot-state-publisher

# install gtsam locally
# cd $GIT_PATH
# git clone https://github.com/borglab/gtsam.git
# cd gtsam && git checkout 4.0.2

# install gtsam globally
cd $SCRIPT_PATH
mkdir lib
cd lib
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
unzip gtsam.zip -d . && cd gtsam-4.0.2/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=ON .. # with march-native
sudo make install -j$(nproc)

# link to workspace and build
cd $WORKSPACE_PATH/src
# ln -s $GIT_PATH/gtsam .
ln -s $GIT_PATH/lio-sam .
# catkin build gtsam
catkin build lio_sam
