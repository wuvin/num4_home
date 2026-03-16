<!--
---
File: num4_home/bin/m4-docker/README.md
Contact: wu.kevi@northeastern.edu
Last Modified: March 16, 2026
---
-->

# num4_home/bin/m4-docker

Build scripts and Dockerfiles from `m4-docker` repository that have been
modified, as part of first (successful) build of (eventual) `num4:ROS2-Humble`
image using `m4-autonomy` repositories and its sub-modules (including
`m4-docker`).

This README contains notes on what was changed in what file to get the build to
complete. Additional notes may also be added here later.

---

## Changes

Applicable files include:
- Build-Scripts/build-control-sensors-livo.sh
- Dockerfiles/Dockerfile.base
- Dockerfiles/Dockerfile.mavtools
- Dockerfiles/Dockerfile.realsense
- Dockerfiles/Dockerfile.livox
- Dockerfiles/Dockerfile.fastlivo2

### Build-Scripts/build-control-sensors-livo.sh

- Add `--network=host` to every `docker build` block
- Refresh expired GPG key
- Add `detect_arch()` function and `ARCH="($detect_arch)` call

### Dockerfiles/Dockerfile.base

- Use PyPI as fallback when encountering `/jp6/cu122/...` package errors

### Dockerfiles/Dockerfile.mavtools

- Include `apt-get` install of `ros-humble-rosidl-default-generator` package

### Dockerfiles/Dockerfile.realsense

- Resolve OpenCV version conflict with that in dustynv Jetson base image
    * Add keys to skip
    * Build `cv_bridge` from source instead against existing OpenCV (4.8.1)
- Add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `setup.sh` run block

### Dockerfiles/Dockerfile.livox

- Add missing ROS packages in `apt` install block:
    * `ros-humble-ament-cmake-auto`
    * `libpcl-dev`
    * `ros-humble-pcl-conversions`
- Add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `setup.sh` run block

TBD
- Change `rm -rf Livox-SDK2` to `rm -rf /opt/Livox-SDK2` to actually remove

### Dockerfiles/Dockerfile.fastlivo2

- Remove `libopencv-dev` from `apt` install block (instead, using OpenCV 4.8.1)
- Add missing ROS packages in `apt` install block:
    * `ros-humble-visualization-msgs`
    * `ros-humble-pcl-ros`
- Source RealSense and Livox workspaces required for FAST-LIVO2 colcon build
- Add skip-keys related to `cv_bridge` and OpenCV, as before
- Strip x86-specific SSE/MMX compiler flags that have no meaning in ARM; I think
  this should just be the x86 SIMD flags from CMakeLists
- Check tag `1.22.10` of Sophus instead, which was version used in previous
  image (i.e., Kamalnath's) with functional FAST-LIVO2

---
