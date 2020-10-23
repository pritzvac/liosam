#!/bin/bash

# DESCRIPTION:
# This script will build GTSAM library from source.
# Make sure that the building flags are correctly set to the workspaces' flags of your packages (default flags correlate with mrs_workspace)
# Do not forget to require the specific PCL_VERSION in your CMakeLists.txt

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

# Releases can be found at: https://github.com/borglab/gtsam
GTSAM_VERSION=4.0.3

# IMPORTANT: These variables should match the settings of your catkin workspace
PROFILE="RelWithDebInfo" # RelWithDebInfo, Release, Debug
BUILD_WITH_MARCH_NATIVE=true
CMAKE_STANDARD=17

# The install directory
INSTALL_DIR=/usr

# Build with march native?
if $BUILD_WITH_MARCH_NATIVE; then
  CMAKE_MARCH_NATIVE="-march=native"
else
  CMAKE_MARCH_NATIVE=""
fi

# Defaults taken from mrs_workspace building flags
BUILD_FLAGS_GENERAL=( 
              -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR
              -DBUILD_apps=ON
              -DBUILD_examples=ON
              -DCMAKE_BUILD_TYPE=$PROFILE
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
              -DCMAKE_CXX_FLAGS="-std=c++17 -fno-diagnostics-color $CMAKE_MARCH_NATIVE" 
              -DCMAKE_C_FLAGS="-fno-diagnostics-color $CMAKE_MARCH_NATIVE"
            )

# Profile-dependent flags
if [[ "$PROFILE" == "RelWithDebInfo" ]]; then
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O2 -g"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O2 -g")
elif [[ "$PROFILE" == "Release" ]]; then
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O3"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O3")
else
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O0 -g"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O0 -g")
fi

# checkout GTSAM_VERSION (ideally stable)
cd $GIT_PATH
[ ! -d "gtsam" ] && git clone https://github.com/borglab/gtsam.git # clone if was not cloned before
cd $GIT_PATH/gtsam && git checkout $GTSAM_VERSION

# go to the build folder
# [ -d "build" ] && rm -rf build # delete if exists
# mkdir build && cd build
cd $GIT_PATH/gtsam
[ ! -d "build" ] && mkdir build # create if does not exists

# install
cd $GIT_PATH/gtsam/build
cmake "${BUILD_FLAGS_GENERAL[@]}" "${BUILD_FLAGS_PROFILE[@]}" ../
echo "Building GTSAM with cmake flags: ${BUILD_FLAGS_GENERAL[@]} ${BUILD_FLAGS_PROFILE[@]}"
echo "This process OFTEN FAILS due to an internal compiler error. In such case, RUN AGAIN the script: $SCRIPT_PATH/install_gtsam.sh"
echo "Building will start in 5s"
sleep 5
sudo make -j$[$(nproc) - 1] install
