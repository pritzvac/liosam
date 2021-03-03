#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

if [[ $# -eq 1 ]]
then
  GTSAM_PATH=$1
else
  GTSAM_PATH=/tmp
fi

[ "$distro" = "18.04" ] && GTSAM_VERSION=4.0.2
[ "$distro" = "20.04" ] && GTSAM_VERSION=4.0.3

if [ ! -d $GTSAM_PATH/gtsam-$GTSAM_VERSION ]
then
  wget -O $GTSAM_PATH/gtsam.zip https://github.com/borglab/gtsam/archive/$GTSAM_VERSION.zip
  cd $GTSAM_PATH && unzip gtsam.zip -d $GTSAM_PATH
fi

cd $GTSAM_PATH/gtsam-$GTSAM_VERSION/

[ ! -d "build" ] && mkdir build
cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
make -j4
sudo make install

# WORKSPACE_PATH=$GIT_PATH/workspace

# # Releases can be found at: https://github.com/borglab/gtsam
# GTSAM_VERSION=4.0.3

# # checkout GTSAM_VERSION (ideally stable)
# cd $GIT_PATH
# [ ! -d "gtsam" ] && git clone https://github.com/borglab/gtsam.git # clone if was not cloned before
# cd $GIT_PATH/gtsam && git checkout $GTSAM_VERSION

# # link gtsam to workspace
# ln -s $GIT_PATH/gtsam $WORKSPACE_PATH/src/.
