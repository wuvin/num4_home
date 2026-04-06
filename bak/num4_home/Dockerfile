# (arm64 only) Use L4T base
FROM dustynv/ros:humble-desktop-l4t-r36.4.0
# (NOTE: dustynv installs most preqs, incl: ROS2 deps, Python packages)

# Prevent prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Clean up old repos
RUN rm -f /etc/apt/sources.list.d/ros2*.list

# Get initial dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
    
# Get ROS2 and Python dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    vim \
    build-essential \
    cmake \
    python3-rosdep \
    network-manager \
    iputils-ping \
    iproute2 \
    net-tools \
    lsof \
    usbutils \
    && rm -rf /var/lib/apt/lists/*
# (dustynv already handles most of this)

# Possible missing runtime dependencies for SDK?
#RUN apt-get update && apt-get install -y \
#    libudns-dev \
#    libpcap-dev \
#    net-tools \
#    lsof \
#    && rm -rf /var/lib/apt/lists/*

# Create directory for building packages and drivers
WORKDIR /ros2_ws

# Install foxglove_bridge
RUN mkdir src/ \
    && git clone https://github.com/foxglove/ros-foxglove-bridge.git src/ros-foxglove-bridge \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get install ros-humble-ament-cmake ros-humble-rosidl-default-generators \
    && rosdep fix-permissions && rosdep install --ignore-src --default-yes --from-path src \
    && colcon build --event-handlers console_direct+ --symlink-install

# Install Livox-SDK2 (needed for livox_ros_driver2)
RUN cd src/ \
    && git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && mkdir ./Livox-SDK2/build \
    && cd ./Livox-SDK2/build \
    && cmake .. && make -j \
    && make -j$(($(nproc)-1)) install

# Install livox_ros_driver2 (manually perform build.sh)
RUN cd src/ \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ros_driver2 \
    && cd livox_ros_driver2 \
    && cp -f package_ROS2.xml package.xml \
    && cp -rf launch_ROS2/ launch/ \
    && cd ../.. \
    && source "/opt/ros/humble/install/setup.bash" \
    && colcon build --symlink-install --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS="humble" --packages-select livox_ros_driver2
# (developer warning: Policy CMP0144/0167/0148 is not set ...
#  ... Use the cmake_policy command to set the policy and suppress this warning)
   
# Fix MID360 config after installation
RUN cd src/livox_ros_driver2/config/ \
    && cp -f MID360_config.json MID360_config.json.bak \
    && sed -i 's/"192.168.1.5"/"192.168.1.50"/g' MID360_config.json \
    && sed -i 's/"192.168.1.12"/"192.168.1.189"/g' MID360_config.json
    
# Alright let's try this
RUN wget https://raw.githubusercontent.com/IntelRealSense/librealsense/refs/heads/master/scripts/libuvc_installation.sh \
    && chmod +x libuvc_installation.sh && ./libuvc_installation.sh
    
    
## Install librealsense2 for Jetson (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
#RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
#    && sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
#    && sudo apt-get install librealsense2-utils -y \
#    && sudo apt-get install librealsense2-dev -y
    
# Install realsense-ros and diagnostic_updater from source
RUN cd src \
    && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master \
    && git clone https://github.com/ros/diagnostics.git -b ros2 \
    && cd ../ \
    && apt-get update \
    && apt-get install -y \
        python3-tqdm \
        libapr1-dev \
        libaprutil1-dev \
    && rm -rf /var/lib/apt/lists/* \
    # Build everything together
    && /bin/bash -c "source /opt/ros/humble/install/setup.bash && \
        colcon build --symlink-install \
        --packages-up-to realsense2_camera \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"
        
# Set up bashrc
RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
# (dustynv already sources initial setup for ROS 2 built from source in /opt/ros/humble/install/...)

# Create directory for external mount
WORKDIR /num4_ws

# Bring color to the world
#RUN echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> ~/.bashrc && \
RUN echo "export PS1='\\[\\e[1;35m\\]\\u@\\h:\\[\\e[1;34m\\]\\w\\[\\e[0m\\]\\$ '" >> ~/.bashrc && \
    echo 'alias ls="ls --color=auto"' >> ~/.bashrc && \
    echo 'alias ll="ls -alF"' >> ~/.bashrc && \
    echo 'alias grep="grep --color=auto"' >> ~/.bashrc

# Set default shell and command
SHELL ["/bin/bash", "-c"]

CMD ["bash"]
