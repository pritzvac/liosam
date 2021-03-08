#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

# get the path to this script
SCRIPT_PATH=`dirname "$0"`
SCRIPT_PATH=`( cd "$MY_PATH" && pwd )`

if [[ $# -eq 1 ]]
then
  WORKSPACE_PATH=$1
else
  WORKSPACE_PATH=$HOME/workspace
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
  ros-$ROS_DISTRO-imu-complementary-filter\
  libmetis-dev

$SCRIPT_PATH/install_gtsam.sh $WORKSPACE_PATH
# $SCRIPT_PATH/install_pcl.sh

# cd $WORKSPACE_PATH
# catkin build
