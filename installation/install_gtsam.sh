#!/bin/bash

wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
cd ~/Downloads/gtsam-4.0.2/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
sudo make install -j4

# WORKSPACE_PATH=$GIT_PATH/workspace

# # Releases can be found at: https://github.com/borglab/gtsam
# GTSAM_VERSION=4.0.3

# # checkout GTSAM_VERSION (ideally stable)
# cd $GIT_PATH
# [ ! -d "gtsam" ] && git clone https://github.com/borglab/gtsam.git # clone if was not cloned before
# cd $GIT_PATH/gtsam && git checkout $GTSAM_VERSION

# # link gtsam to workspace
# ln -s $GIT_PATH/gtsam $WORKSPACE_PATH/src/.
