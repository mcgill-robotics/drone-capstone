# Base image with CUDA + ROS Humble + JetPack 6.1 (L4T 36.4.0)
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

# Environment
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# NOTE: don't force OpenCV_DIR here — let CMake find the one that comes with the image
# (it’s already compiled against CUDA 12 inside this base image)

# GPG key update for ROS (kept as you had it)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install apt packages
# IMPORTANT: we only install *generic* and *non-OpenCV* packages here.
# ROS "desktop" stuff is already included in the base image, so we don't re-install ROS debs
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
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
        # RealSense / USB deps
        udev \
        libusb-1.0-0-dev \
        libusb-1.0-0 \
        libssl-dev \
        libgtk-3-dev \
        libgtk-3-0 \
        libglfw3-dev \
        libglfw3 \
        # Colcon / build tooling
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
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
    && rm -rf /var/lib/apt/lists/*

# (Optional but usually helpful)
# Initialize rosdep (inside the image, one-time)
RUN rosdep init 2>/dev/null || true && \
    rosdep update

# Build librealsense 2.53.1 (T265 last working version)
# We DISABLE examples to avoid any hard dependency on OpenCV here.
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

# Clone and build your ROS workspace
# (Assumes drone-capstone has a ros2_ws with correct package.xml/CMakeLists.txt)
RUN git clone https://github.com/mcgill-robotics/drone-capstone /drone-capstone && \
    cd /drone-capstone/ros2_ws && \
    source /opt/ros/humble/setup.bash \
    # If you need additional deps, you can uncomment the rosdep line:
    # rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

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

# Shell setup for ROS + workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /drone-capstone/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "source /ros_entrypoint.sh" >> ~/.bashrc && \
    echo "tmux" >> ~/.bashrc
