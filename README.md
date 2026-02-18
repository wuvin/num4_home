<!--
---
File: config/README.md
Contact: wu.kevi@northeastern.edu
Last Modified: February 18, 2026
---
-->

# num4_home/

Directory with contents for the operation of NUM4.

---

## ./bin/

Utilities that exist above ROS 2 workspace and outside Docker container.

### Scripts

| NAME | DESC |
| :--- | :--: |
| build_Dockerfile.sh | Sample command used to build from Dockerfile on remote |
| fix_jetson_routing_dns.sh | Allow remote to use connection shared by client |
| loggercmd.sh | Script sourced by `/etc/bash.bashrc` to record CLI executions |
| loggerin.service | Script in `/etc/systemd/system/` that tells `systemd` to execute `loggerin.sh` when Jetson starts |
| loggerin.sh | Script that logs to `/home/$USER/Documents/log.txt` upon certain events |
| route_network_interface.sh | Allow client to share its internet connection |
| setup_num4_data.sh | Client-side script to set up Docker terminals on remote for data collection |
| ssh_port_forward_u2d2.sh | Failed attempt at SSH port forwarding for DYNAMIXEL Wizard 2.0 to be used between client and remote |

### Forwarding Internet Connection

Provide laptop's internet access to Jetson. If the Jetson connection is wireless
or via WiFi, then the laptop's internet connection cannot also be wireless; that
is, the internet and Jetson connections cannot be through the same interface.

---

## ./num4_ws/

ROS 2 workspace directory to be mounted inside of Docker container.

### ./num4_ws/bin/

Utilities to be used within Docker container.

| NAME | DESC |
| :--- | :--: |
| run_num4_container.sh | Runs a Docker container with a specific ROS 2 set-up |
| show_ros2_examples.sh | Examples of what gets executed in a Docker container |

### ./num4_ws/src/

Custom ROS 2 packages.

| NAME | DESC |
| :--- | :--: |
| livox_bringup | Usage of and configs for Livox MID360 |
| pc2image | Project LiDAR point clouds onto camera images |
| realsense2_bringup | Usage of and configs for RealSense D455 |

External ROS 2 packages.

| NAME | DESC |
| :--- | :--: |
| imu_tools | Filters (including Madgwick) and tools for IMU sensors |
| livox_ros_driver2 | Livox-provided custom message formats |
| livox_to_pointcloud2 | Routine to convert into PointCloud2 format |
| realsense2_camera_msgs | Custom message formats for camera properties |
| ros-foxglove-bridge | Bridge needed to interface with Foxglove |
| rosx_introspection | Parse ROS messages without compile-time info |

---