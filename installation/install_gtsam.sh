#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"
[ "$distro" = "18.04" ] && GTSAM_VERSION=4.0.2
[ "$distro" = "20.04" ] && GTSAM_VERSION=4.0.3

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
GTSAM_PATH=$SCRIPT_PATH/../lib

[ ! -d $GTSAM_PATH ] && mkdir -p $GTSAM_PATH
cd $GTSAM_PATH

if [ ! -d $GTSAM_PATH/gtsam-$GTSAM_VERSION ]
then
  wget -O $GTSAM_PATH/gtsam.zip https://github.com/borglab/gtsam/archive/$GTSAM_VERSION.zip
  cd $GTSAM_PATH && unzip gtsam.zip -d $GTSAM_PATH
  rm -f gtsam.zip
fi

cd $GTSAM_PATH/gtsam-$GTSAM_VERSION/

[ ! -d "build" ] && mkdir build
cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
make -j$[$(nproc) - 1]
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
