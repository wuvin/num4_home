# Setting up Client-Remote connection ("bridge") with common Server
Client: `ssh -L 2222:<remote-server-ip>:22 <server-user>:<client-server-ip>`
Client (new terminal): `ssh -Y <remote-user>@localhost -p 2222`

# Installing apps through bridge
Client: `sudo apt-get --download-only --reinstall install vim:arm64`
Client: `ls /var/cache/apt/archives` should return corresponding *.deb
Client: `scp -P 2222 /var/cache/apt/archives/*.deb <remote-user>@localhost:<..>`
Remote: `docker cp <..> <docker-name>:<path>`
Container: `sudo dpkg -i *.deb`

NOTE: Jetson Orin is arm64, NOT amd64, so make sure *.deb is compatible.
This can be done with `sudo dpkg --add-architecture arm64`, followed by
updating with `sudo apt-get update`, and specifying ':arm64' for the package.
If this does not work still, then you have to locate the correct deb elsewhere.

# Using SOCKS proxy to provide internet to Remote
Remote: `ssh -D 8080 <server-user>@<server-remote-ip>` (keep this terminal open)
Remote (new terminal):
    `sudo apt -o Acquire::http::proxy="socks5h://127.0.0.1:8080/" \`
    `-o Acquire:https::proxy="socks5h://127.0.0.1:8080/" install vim`
This works for `apt`.  For another thing that requires internet, may need
something else.

End first Remote terminal to close SOCKS proxy.

-----

Random sha256 from image copying:
```
num4@num4:~$ docker commit num4_ros2_humble_test_new wuvin:v1
sha256:ce743089f131192e6034696faf97f341896bb51712e3def71bf06707765bc931
```

```
num4@num4:~$ docker commit num4_ros2_v1 wuvin:v1
sha256:5b181b2679a1df039be9a52ffa469096cb5e4a0a542d617f15cd3e2130e640b4
```

```
num4@num4:~/wuvin$ docker commit num4_ros2_v1 wuvin:v1
sha256:b41373b0e8c3c0463e441a479846d934d8af0d8ec69b978aca59388318324267
```

```
num4@num4:~$ docker commit num4_ros2_v1 wuvin:v2
sha256:9d2143fae29318678f34a7e9f7f18379a6fdd9c1be588ca46f99c4e1e7e12136
```

-----

Must ensure only one active network interface for CycloneDDS to work

nmcli radio wifi off
nmcli radio wifi on
nmcli device

Actually, after a lot of testing...

Ethernet + FastDDS (rmw_fastrtps_cpp) seems to work fine, though ros2 topic
list times out on client side, and not on remote side, on domain 9 (but still
can see each other's topics in domain 9)

Ethernet + CycloneDDS seems to fail regardless of domain ID

Previously, CycloneDDS worked over shared network on WiFi.  Maybe issue is DDS
discovery issue that's exacerbated with CycloneDDS over ethernet?  Cyclone is
pickier; Fast is more robust and has autoselection, so Cyclone is prob binding
to wrong network interface

-----

Foxglove terminal

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

(Foxglove > open connection)

-----

2x laptop terminal

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42

-----

ssh into num4

cd wuvin
./run_container.sh

on another terminal: docker exec -it num4_ros2_v1 bash

main:
cd m4_ws/src/m4-sensors/
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run realsense2_camera realsense2_camera_node

verify topics appear + foxglove shows

side:
cd m4_ws/src/ws_livox/
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

verify again

-----

move terminals to

/mnt/kwu/projects/ros2_ws_livox

then re-source (need custom msgs)

-----

(terminal 1)
ros2 bag record -s mcap -o run20250828_cal_imu /camera/camera/accel/sample /camera/camera/accel/imu_info /camera/camera/accel/metadata /camera/camera/gyro/sample /camera/camera/gyro/imu_info /camera/camera/gyro/metadata /livox/imu

(terminal 2)
ros2 bag record -s mcap -o run20250828_cal_dpt /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info

(terminal 3)
ros2 bag record -s mcap -o run20250828_cal_lid /livox/lidar

-----

on sender (probably should also do receiver, get both sides, just to be safe)

export CYCLONEDDS_URI=/home/num4/wuvin/cyclonedds2.xml
    leave autodetermine to True to show which is default; else, set it False then specify name="wls..."

for some reason, remote can only publish to client, not vice versa, for now, on cyclonedds -- maybe because of Domain ID or network (wifi)?

NEVERMIND.  Topic isn't visible through ros2 topic list for some reason, but able to run ros2 run demo_nodes_cpp talker/listener either way

BUT ros2 multicast send/receive doesn't seem to work  (need to check again on new session)

-----

foxglove:
source /opt/ros/humble/setup.bash \
&& ros2 launch foxglove_bridge foxglove_bridge_launch.xml

(Foxglove client > open connection > ws:\\num4.local)

camera:
cd m4_ws/src/m4-sensors \
&& source /opt/ros/humble/setup.bash \
&& source install/setup.bash \
&& export CYCLONEDDS_URI=/home/num4/wuvin/cyclonedds2.xml \
&& ros2 launch realsense2_bringup NOPEd435i.launch.py

lidar:
cd m4_ws/src/ws_livox \
&& source /opt/ros/humble/setup.bash \
&& source install/setup.bash \
&& export CYCLONEDDS_URI=/home/num4/wuvin/cyclonedds2.xml \
&& ros2 launch livox_ros_driver2 rviz_MID360_launch.py
