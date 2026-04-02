# Base image with CUDA + ROS Humble + JetPack 6.1 (L4T 36.4.0)
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

# Environment
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PIP_INDEX_URL=https://pypi.org/simple \
    PIP_EXTRA_INDEX_URL=

# Remove Jetson pip override
RUN rm -rf /root/.config/pip || true

# GPG key update for ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install apt packages
RUN apt-get update -qq && \
    apt-get remove -y opencv-dev opencv-libs opencv-main 'libopencv*' || true && \
    apt-get autoremove -y && \
    apt-get install -y -qq --no-install-recommends \
        -o Dpkg::Options::="--force-overwrite" \
        git \
        tmux \
        vim \
        cmake \
        build-essential \
        pkg-config \
        python3-dev \
        python3-pip \
        wget \
        curl \
        udev \
        libusb-1.0-0-dev \
        libusb-1.0-0 \
        libssl-dev \
        libgtk-3-dev \
        libgtk-3-0 \
        libglfw3-dev \
        libglfw3 \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-rospkg-modules \
        ros-${ROS_DISTRO}-ament-cmake-python \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-generator-cpp \
        ros-${ROS_DISTRO}-rosidl-generator-py \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-rosidl-cmake \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
        ros-${ROS_DISTRO}-mavros-msgs \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-pluginlib \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-yaml-cpp-vendor \
        ros-${ROS_DISTRO}-camera-calibration-parsers \
        ros-${ROS_DISTRO}-ament-cmake-* \
        ros-${ROS_DISTRO}-rclcpp-components \
        ros-${ROS_DISTRO}-tf2-* \
        ros-${ROS_DISTRO}-stereo-msgs \
        ros-${ROS_DISTRO}-librealsense2 \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-ros-environment \
        ros-${ROS_DISTRO}-launch-ros \
        ros-${ROS_DISTRO}-launch \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-ros-testing \
        ros-${ROS_DISTRO}-camera-ros \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Install required pip packages
RUN pip3 install --upgrade pip pyros-genmsg setuptools

# Build Micro-XRCE-DDS-Agent for PX4
RUN cd /tmp && \
    git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig /usr/local/lib/ && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Initialize rosdep
RUN rosdep init 2>/dev/null || true && \
    rosdep update

# Build librealsense 2.53.1 (T265 last working version)
RUN cd /tmp && \
    wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.53.1.tar.gz && \
    tar -xzf v2.53.1.tar.gz && \
    cd librealsense-2.53.1 && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=ON \
        -DBUILD_PYTHON_BINDINGS=OFF && \
    make -j"$(nproc)" && \
    make install && \
    cd / && rm -rf /tmp/librealsense-2.53.1 /tmp/v2.53.1.tar.gz

# Build PX4 SITL (strip any stray opencv refs from the setup script first)
RUN cd /tmp && \
    git clone https://github.com/mcgill-robotics/PX4-Autopilot.git --recursive && \
    cd PX4-Autopilot && \
    sed -i 's/libopencv[^ ]*//g' Tools/setup/ubuntu.sh && \
    bash ./Tools/setup/ubuntu.sh && \
    make px4_sitl && \
    cd / && rm -rf /tmp/PX4-Autopilot

# Install gz dev headers (OSRF repo required; runtime libs already present but headers are not)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        libgz-transport13-dev \
        libgz-msgs10-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone and build ROS workspace
RUN cd /tmp && \
    git clone https://github.com/mcgill-robotics/drone-capstone -b ros2hotfix_gazebofail && \
    source /opt/ros/humble/setup.bash && \
    cd drone-capstone/ros2_ws && \
    colcon build && \
    source install/setup.bash && \
    cd / && rm -rf /tmp/drone-capstone

# udev rules for RealSense cameras
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b37", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/99-realsense-libusb.rules && \
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules

# Use bash for later RUNs
SHELL ["/bin/bash", "-c"]

# Shell setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /drone-capstone/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "source /ros_entrypoint.sh" >> ~/.bashrc && \
    echo "tmux" >> ~/.bashrc
