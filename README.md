# Lidar Inertial Odometry via Smoothing and Mapping (LIO-SAM)

This is MRS version of the LIO-SAM implementation forked from [https://github.com/TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).

### The [original README](https://github.com/TixiaoShan/LIO-SAM) contains
  - description of the method,
  - description of the implementation,
  - multimedia materials showing the performance of LIO-SAM,
  - data preparation manuals,
  - list of dependencies,
  - paper references, and
  - list of possible bugs.
 
### MRS version has changed
  - WIP: test, nodelet, prepare for use on MAVs

## Installation
Run `./installation/install.sh`, which:
  - installs dependencies ([GTSAM](https://github.com/borglab/gtsam) ver. 4.0.2, Intel TBB, and PCL),
  - links GTSAM and LIO-SAM to `~/workspace/src` by default, and
  - builds GTSAM (may take even over 10 minutes) and LIO-SAM. 
