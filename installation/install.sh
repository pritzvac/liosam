#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

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
sudo apt-get -y install \
  libtbb-dev \
  ros-$ROS_DISTRO-pcl-* \
  libpcl-* \
  ros-$ROS_DISTRO-navigation \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-imu-complementary-filter

$SCRIPT_PATH/install_gtsam.sh $WORKSPACE_PATH
# $SCRIPT_PATH/install_pcl.sh

# cd $WORKSPACE_PATH
# catkin build
