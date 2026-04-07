# ============================================================
#  McGill Robotics — Drone Capstone
#  Clean build from L4T base: no pre-installed OpenCV / ROS
#  Target: JetPack 6.1  (L4T 36.4.0, CUDA 12.6, aarch64)
# ============================================================
FROM nvcr.io/nvidia/l4t-base:r36.4.0

# ── Environment ─────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    # librealsense 2.53.1 is the last version with T265 support
    LIBREALSENSE_VERSION=2.53.1 \
    # OpenCV 4.8.1 – stable, ROS-compatible, CUDA-enabled
    OPENCV_VERSION=4.8.1 \
    # Micro-XRCE-DDS-Agent tag
    XRCE_DDS_TAG=v2.4.3 \
    # All from-source installs go here so they win over any stray apt paths
    CMAKE_INSTALL_PREFIX=/usr/local

# ── Nuke every OpenCV package that might be on the base image ──
RUN dpkg -l | grep -i opencv | awk '{print $2}' | xargs apt-get purge -y 2>/dev/null || true && \
    apt-get autoremove -y 2>/dev/null || true && \
    rm -f /usr/lib/aarch64-linux-gnu/libopencv* \
          /usr/lib/aarch64-linux-gnu/pkgconfig/opencv* \
          /usr/local/lib/libopencv* \
          /usr/local/lib/pkgconfig/opencv* \
          /usr/include/opencv2 2>/dev/null || true && \
    ldconfig

# ── Base system deps ─────────────────────────────────────────
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
        # Build toolchain
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        git \
        wget \
        curl \
        unzip \
        # Python
        python3 \
        python3-dev \
        python3-pip \
        python3-numpy \
        # USB / serial
        libusb-1.0-0-dev \
        libusb-1.0-0 \
        udev \
        # SSL / crypto
        libssl-dev \
        # GTK / GL (needed by OpenCV highgui and RealSense viewer)
        libgtk-3-dev \
        libgtk-3-0 \
        libglfw3-dev \
        libglfw3 \
        libgles2-mesa-dev \
        # Image / video codec libs (OpenCV)
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libwebp-dev \
        libopenexr-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev \
        libatlas-base-dev \
        gfortran \
        # Misc
        tmux \
        vim \
        lsb-release \
        gnupg2 \
        software-properties-common \
        locales \
    && rm -rf /var/lib/apt/lists/*

# ── OpenCV 4.8.1 from source (CUDA-enabled, no GTK issues) ──
#    We build contrib too so ArUco / AprilTag detector is available.
RUN cd /tmp && \
    wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz \
         -O opencv-${OPENCV_VERSION}.tar.gz && \
    wget -q https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.tar.gz \
         -O opencv_contrib-${OPENCV_VERSION}.tar.gz && \
    tar -xzf opencv-${OPENCV_VERSION}.tar.gz && \
    tar -xzf opencv_contrib-${OPENCV_VERSION}.tar.gz && \
    mkdir -p opencv-${OPENCV_VERSION}/build && \
    cd opencv-${OPENCV_VERSION}/build && \
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
        # CUDA – uses the CUDA already on the L4T image
        -DWITH_CUDA=ON \
        -DCUDA_ARCH_BIN="8.7" \
        -DWITH_CUBLAS=ON \
        -DWITH_CUDNN=OFF \
        # Python bindings (useful for debugging)
        -DBUILD_opencv_python3=ON \
        -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
        # No GTK popups in headless Docker
        -DWITH_GTK=OFF \
        -DWITH_QT=OFF \
        # Codecs
        -DWITH_FFMPEG=ON \
        -DWITH_V4L=ON \
        -DWITH_LIBV4L=ON \
        # Strip stuff we don't need to keep build time reasonable
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCS=OFF \
        -DBUILD_opencv_apps=OFF \
    && ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd / && rm -rf /tmp/opencv-${OPENCV_VERSION} \
                   /tmp/opencv_contrib-${OPENCV_VERSION} \
                   /tmp/opencv-${OPENCV_VERSION}.tar.gz \
                   /tmp/opencv_contrib-${OPENCV_VERSION}.tar.gz

# ── librealsense 2.53.1 from source (last T265 release) ─────
#    Do NOT install ros-humble-librealsense2 from apt — it brings 2.56.x
#    and will silently override our /usr/local install at runtime.
RUN cd /tmp && \
    wget -q https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBREALSENSE_VERSION}.tar.gz \
         -O librealsense-${LIBREALSENSE_VERSION}.tar.gz && \
    tar -xzf librealsense-${LIBREALSENSE_VERSION}.tar.gz && \
    mkdir -p librealsense-${LIBREALSENSE_VERSION}/build && \
    cd librealsense-${LIBREALSENSE_VERSION}/build && \
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        # OpenCV was just installed to /usr/local — point at it explicitly
        -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=ON \
        -DBUILD_PYTHON_BINDINGS=OFF \
        # CUDA for depth processing acceleration
        -DBUILD_WITH_CUDA=ON \
    && ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd / && rm -rf /tmp/librealsense-${LIBREALSENSE_VERSION} \
                   /tmp/librealsense-${LIBREALSENSE_VERSION}.tar.gz

# udev rules for RealSense (T265 = 8086:0b37, D435 = 8086:0b07, etc.)
RUN for RULE in \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b37", MODE="0666", GROUP="plugdev"' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE="0666", GROUP="plugdev"' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE="0666", GROUP="plugdev"' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666", GROUP="plugdev"' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE="0666", GROUP="plugdev"' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666", GROUP="plugdev"' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"'; \
    do echo "$RULE"; done > /etc/udev/rules.d/99-realsense-libusb.rules

# ── ROS 2 Humble (official OSRF debs, no ros-humble-librealsense2) ──
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-ament-cmake \
        ros-humble-ament-cmake-python \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-generator-cpp \
        ros-humble-rosidl-generator-py \
        ros-humble-rosidl-cmake \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        ros-humble-mavros-msgs \
        ros-humble-diagnostic-updater \
        ros-humble-message-filters \
        ros-humble-pluginlib \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-yaml-cpp-vendor \
        ros-humble-camera-calibration-parsers \
        ros-humble-rclcpp-components \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-sensor-msgs \
        ros-humble-stereo-msgs \
        ros-humble-nav-msgs \
        ros-humble-ros-environment \
        ros-humble-launch-ros \
        ros-humble-launch \
        ros-humble-std-srvs \
        ros-humble-launch-testing \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-ros-testing \
        ros-humble-camera-ros \
        ros-humble-xacro \
        ros-humble-ros-gz \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-rospkg-modules \
        python3-catkin-pkg \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic dev headers (runtime libs come with ros-humble-ros-gz)
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

# ── realsense-ros from source (must match librealsense 2.53.1) ──
#    The apt package ros-humble-realsense2-* would pull librealsense 2.56.x
#    as a hard dep — we build from the matching tag instead.
RUN source /opt/ros/humble/setup.bash && \
    pip3 install --upgrade pip pyros-genmsg setuptools && \
    rosdep init 2>/dev/null || true && \
    rosdep update && \
    mkdir -p /opt/realsense_ros_ws/src && \
    cd /opt/realsense_ros_ws/src && \
    git clone --branch 4.51.1 --depth 1 \
        https://github.com/IntelRealSense/realsense-ros.git && \
    cd /opt/realsense_ros_ws && \
    # Tell CMake where our from-source librealsense lives
    colcon build \
        --cmake-args \
            -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
            -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
            -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# ── Micro-XRCE-DDS-Agent (PX4 ↔ ROS 2 bridge) ──────────────
RUN cd /tmp && \
    git clone --depth 1 -b ${XRCE_DDS_TAG} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    mkdir -p Micro-XRCE-DDS-Agent/build && \
    cd Micro-XRCE-DDS-Agent/build && \
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig /usr/local/lib/ && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

# ── Serial reconnect supervisor for Micro-XRCE-DDS-Agent ────
#
#  The agent exits when the serial port disappears (FC reboot, cable wiggle).
#  This wrapper loops forever, waiting for the device node before (re)launching
#  the agent.  Call it instead of MicroXRCEAgent directly:
#
#    /usr/local/bin/xrce_agent_serial.sh /dev/ttyTHS1 921600
#
COPY xrce_agent_serial.sh /usr/local/bin/xrce_agent_serial.sh
RUN chmod +x /usr/local/bin/xrce_agent_serial.sh

# ── PX4 SITL ────────────────────────────────────────────────
RUN cd / && \
    git clone --recursive https://github.com/mcgill-robotics/PX4-Autopilot.git && \
    cd /PX4-Autopilot && \
    # Strip any lingering opencv references from the setup helper
    sed -i 's/libopencv[^ ]*//g' Tools/setup/ubuntu.sh && \
    # git safe.directory fix (build runs as root)
    git config --global --add safe.directory /PX4-Autopilot && \
    git submodule foreach --recursive \
        'git config --global --add safe.directory $(pwd) 2>/dev/null || true' && \
    bash ./Tools/setup/ubuntu.sh --no-nuttx && \
    make px4_sitl

# ── Drone capstone ROS 2 workspace ─────────────────────────
RUN source /opt/ros/humble/setup.bash && \
    source /opt/realsense_ros_ws/install/setup.bash && \
    git clone https://github.com/mcgill-robotics/drone-capstone /drone-capstone && \
    cd /drone-capstone/ros2_ws && \
    colcon build \
        --cmake-args \
            -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
            -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
            -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# ── Shell setup ─────────────────────────────────────────────
SHELL ["/bin/bash", "-c"]

RUN cat >> ~/.bashrc << 'EOF'
source /opt/ros/humble/setup.bash
source /opt/realsense_ros_ws/install/setup.bash
source /drone-capstone/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Ensure our from-source libs are found first
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# ── Aliases ──────────────────────────────────────────────────
# Full clean rebuild of capstone workspace
alias cb='cd /drone-capstone/ros2_ws && rm -rf build install log && \
    colcon build \
        --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
                     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
                     -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash'

# Incremental rebuild
alias b='cd /drone-capstone/ros2_ws && \
    colcon build \
        --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
                     -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
                     -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash'

# PX4 SITL (with GUI)
alias martin='cd /PX4-Autopilot && \
    PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam_down NAV_DLL_ACT=0'

# PX4 SITL (headless)
alias jp='cd /PX4-Autopilot && \
    PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam_down NAV_DLL_ACT=0 HEADLESS=1'

# Micro-XRCE-DDS-Agent over serial with auto-reconnect
# Usage:  xrce_serial [device] [baudrate]
alias xrce_serial='xrce_agent_serial.sh /dev/ttyTHS1 921600'

# Precision landing mission
alias mission='cd /drone-capstone/ros2_ws && source install/setup.bash && \
    ros2 run precision_land_apriltag precision_land_apriltag_node'

alias missionb='cd /drone-capstone/ros2_ws && \
    colcon build --packages-select precision_land_apriltag && \
    source install/setup.bash && \
    ros2 run precision_land_apriltag precision_land_apriltag_node'

alias viz='ros2 run rqt_image_view rqt_image_view --clear-config'

tmux
EOF

# ── Entrypoint ──────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
