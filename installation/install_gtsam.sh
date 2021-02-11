#!/bin/bash

if [[ $# -eq 1 ]]
then
  WORKSPACE_PATH=$1
else
  WORKSPACE_PATH=$GIT_PATH/workspace
fi

# Releases can be found at: https://github.com/borglab/gtsam
GTSAM_VERSION=4.0.3

# checkout GTSAM_VERSION (ideally stable)
cd $GIT_PATH
[ ! -d "gtsam" ] && git clone https://github.com/borglab/gtsam.git # clone if was not cloned before
cd $GIT_PATH/gtsam && git checkout $GTSAM_VERSION

# link gtsam to workspace
ln -s $GIT_PATH/gtsam $WORKSPACE_PATH/src/.
