# ============================================================
#  McGill Robotics — Drone Capstone
#  Base: dustynv ROS Humble + JetPack 6.1 (L4T 36.4.0, aarch64)
# ============================================================
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    LIBREALSENSE_VERSION=2.53.1 \
    OPENCV_VERSION=4.8.1 \
    XRCE_DDS_TAG=v2.4.3

# ── 1. Fix the expired ROS GPG key that ships in dustynv ────────────
#    Must be the very first step — everything else calls apt-get update.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list && \
    find /etc/apt/sources.list.d/ -name '*.list' ! -name 'ros2.list' \
        -exec grep -l 'packages.ros.org' {} \; | xargs rm -f 2>/dev/null || true

# ── 2. Purge ALL pre-installed OpenCV and librealsense ──────────────
#    dustynv ships its own OpenCV; ros-humble-librealsense2 from apt
#    brings librealsense 2.56.x. We remove both and rebuild from source
#    so exactly one version of each exists at a known path.
RUN apt-get update -qq && \
    OPENCV_PKGS=$(dpkg -l 2>/dev/null | grep -i opencv | awk '{print $2}' | tr '\n' ' ') && \
    RS_PKGS=$(dpkg -l 2>/dev/null | grep -iE 'librealsense|realsense' | awk '{print $2}' | tr '\n' ' ') && \
    [ -n "$OPENCV_PKGS" ] && apt-get purge -y --allow-change-held-packages $OPENCV_PKGS || true && \
    [ -n "$RS_PKGS" ]     && apt-get purge -y --allow-change-held-packages $RS_PKGS     || true && \
    apt-get purge -y --allow-change-held-packages \
        ros-humble-librealsense2 \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        2>/dev/null || true && \
    apt-get autoremove -y && \
    find /usr/lib /usr/local/lib \( -name 'libopencv*' -o -name 'librealsense*' \) \
        -delete 2>/dev/null || true && \
    find /usr/include /usr/local/include -maxdepth 2 \
        \( -name 'opencv2' -o -name 'librealsense2' \) \
        -exec rm -rf {} + 2>/dev/null || true && \
    find /usr /usr/local \
        \( -name 'OpenCVConfig*.cmake' -o -name 'realsense2Config*.cmake' \) \
        -delete 2>/dev/null || true && \
    ldconfig && \
    rm -rf /var/lib/apt/lists/*

# ── 3. Build-time system dependencies ───────────────────────────────
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        git \
        wget \
        curl \
        unzip \
        python3-dev \
        python3-pip \
        python3-numpy \
        libusb-1.0-0-dev \
        libusb-1.0-0 \
        udev \
        libssl-dev \
        libglfw3-dev \
        libglfw3 \
        libgles2-mesa-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libwebp-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev \
        libatlas-base-dev \
        gfortran \
        tmux \
        vim \
        lsb-release \
        gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# ── 4. OpenCV 4.8.1 from source (CPU only, contrib for ArUco) ───────
#
#    WHY NO CUDA: opencv_contrib's cudev module uses Thrust tuple
#    overloads removed in CUDA 12.5+. JetPack 6.1 ships CUDA 12.6,
#    so cudaarithm fails with a blockReduce template deduction error.
#    AprilTag/ArUco detection runs fine on CPU — no CUDA needed here.
#    We include opencv_contrib only for the CPU aruco module.
RUN cd /tmp && \
    wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz \
         -O opencv.tar.gz && \
    wget -q https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.tar.gz \
         -O opencv_contrib.tar.gz && \
    tar -xzf opencv.tar.gz && \
    tar -xzf opencv_contrib.tar.gz && \
    # Remove any stale opencv4 include dir left by the purge step —
    # if it exists as a plain file or has wrong permissions,
    # "ninja install" will fail trying to mkdir inside it.
    rm -rf /usr/local/include/opencv4 && \
    mkdir -p opencv-${OPENCV_VERSION}/build && \
    cd opencv-${OPENCV_VERSION}/build && \
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
        -DWITH_CUDA=OFF \
        -DWITH_CUBLAS=OFF \
        -DWITH_CUDNN=OFF \
        -DBUILD_opencv_cudaarithm=OFF \
        -DBUILD_opencv_cudabgsegm=OFF \
        -DBUILD_opencv_cudacodec=OFF \
        -DBUILD_opencv_cudafeatures2d=OFF \
        -DBUILD_opencv_cudafilters=OFF \
        -DBUILD_opencv_cudaimgproc=OFF \
        -DBUILD_opencv_cudalegacy=OFF \
        -DBUILD_opencv_cudaobjdetect=OFF \
        -DBUILD_opencv_cudaoptflow=OFF \
        -DBUILD_opencv_cudastereo=OFF \
        -DBUILD_opencv_cudawarping=OFF \
        -DBUILD_opencv_cudev=OFF \
        -DBUILD_opencv_aruco=ON \
        -DBUILD_opencv_python3=ON \
        -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
        -DWITH_GTK=OFF \
        -DWITH_QT=OFF \
        -DWITH_FFMPEG=ON \
        -DWITH_V4L=ON \
        -DWITH_LIBV4L=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCS=OFF \
        -DBUILD_opencv_apps=OFF \
    && ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd / && rm -rf /tmp/opencv.tar.gz /tmp/opencv_contrib.tar.gz \
                   /tmp/opencv-${OPENCV_VERSION} /tmp/opencv_contrib-${OPENCV_VERSION}

# ── 5. librealsense 2.53.1 from source ──────────────────────────────
#    Last release with T265 support. Never install ros-humble-librealsense2
#    from apt after this — it will silently pull in 2.56.x.
RUN cd /tmp && \
    wget -q https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBREALSENSE_VERSION}.tar.gz \
         -O librealsense.tar.gz && \
    tar -xzf librealsense.tar.gz && \
    mkdir -p librealsense-${LIBREALSENSE_VERSION}/build && \
    cd librealsense-${LIBREALSENSE_VERSION}/build && \
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=ON \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_WITH_CUDA=OFF \
    && ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd / && rm -rf /tmp/librealsense.tar.gz /tmp/librealsense-${LIBREALSENSE_VERSION}

# udev rules for RealSense cameras
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b37", MODE="0666", GROUP="plugdev"' \
        > /etc/udev/rules.d/99-realsense-libusb.rules && \
    for ID in 0ad1 0ad2 0ad3 0ad4 0b07; do \
        echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"8086\", ATTRS{idProduct}==\"${ID}\", MODE=\"0666\", GROUP=\"plugdev\"" \
            >> /etc/udev/rules.d/99-realsense-libusb.rules; \
    done && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"' \
        >> /etc/udev/rules.d/99-realsense-libusb.rules

# ── 6. Additional ROS 2 Humble packages ─────────────────────────────
#    dustynv already has ros-humble-ros-base and most core libs.
#    Do NOT add ros-humble-librealsense2 / realsense2-* here.
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
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
    && rm -rf /var/lib/apt/lists/*

# ── 7. Gazebo dev headers ────────────────────────────────────────────
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

# ── 8. realsense-ros from source ────────────────────────────────────
#    4.51.1 is the last tag tested against librealsense 2.53.x.
#
#    dustynv's ROS Humble is a colcon workspace built under /opt/ros/humble.
#    The cmake package configs live under /opt/ros/humble/install/<pkg>/share,
#    not /opt/ros/humble/share. We must discover the real AMENT_PREFIX_PATH
#    by sourcing setup.bash, then pass every entry as CMAKE_PREFIX_PATH.
#    We also install ros-humble-rosidl-default-generators explicitly in case
#    it's missing from the base image.
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        ros-humble-ament-cmake \
        ros-humble-ament-cmake-python \
    && rm -rf /var/lib/apt/lists/*

RUN unset PIP_INDEX_URL PIP_EXTRA_INDEX_URL && \
    pip3 install --upgrade \
        --index-url https://pypi.org/simple \
        pip pyros-genmsg setuptools && \
    rosdep init 2>/dev/null || true && \
    rosdep update && \
    mkdir -p /opt/realsense_ros_ws/src && \
    cd /opt/realsense_ros_ws/src && \
    git clone --branch 4.51.1 --depth 1 \
        https://github.com/IntelRealSense/realsense-ros.git && \
    cd /opt/realsense_ros_ws && \
    bash -c "\
        source /opt/ros/humble/setup.bash && \
        # AMENT_PREFIX_PATH is now set; convert it to CMAKE_PREFIX_PATH format
        ROS_CMAKE_PREFIX=\$(echo \"\$AMENT_PREFIX_PATH\" | tr ':' ';') && \
        colcon build \
            --cmake-args \
                \"-DCMAKE_PREFIX_PATH=\${ROS_CMAKE_PREFIX}\" \
                -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
                -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
                -DCMAKE_BUILD_TYPE=Release \
    " && \
    rm -rf build log

# ── 9. Micro-XRCE-DDS-Agent ─────────────────────────────────────────
RUN cd /tmp && \
    git clone --depth 1 -b ${XRCE_DDS_TAG} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig /usr/local/lib/ && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

# ── 10. PX4 SITL ─────────────────────────────────────────────────────
RUN git clone --recursive https://github.com/mcgill-robotics/PX4-Autopilot.git /PX4-Autopilot && \
    cd /PX4-Autopilot && \
    git config --global --add safe.directory /PX4-Autopilot && \
    git submodule foreach --recursive \
        'git config --global --add safe.directory "$(pwd)" 2>/dev/null || true' && \
    sed -i 's/libopencv[^ ]*//g' Tools/setup/ubuntu.sh && \
    bash ./Tools/setup/ubuntu.sh --no-nuttx && \
    make px4_sitl

# ── 11. Drone capstone ROS 2 workspace ───────────────────────────────
RUN git clone https://github.com/mcgill-robotics/drone-capstone /drone-capstone && \
    bash -c "\
        source /opt/ros/humble/setup.bash && \
        source /opt/realsense_ros_ws/install/setup.bash && \
        ROS_CMAKE_PREFIX=\$(echo \"\$AMENT_PREFIX_PATH\" | tr ':' ';') && \
        cd /drone-capstone/ros2_ws && \
        colcon build \
            --cmake-args \
                \"-DCMAKE_PREFIX_PATH=\${ROS_CMAKE_PREFIX}\" \
                -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
                -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
                -DCMAKE_BUILD_TYPE=Release \
    " && \
    rm -rf /drone-capstone/ros2_ws/build /drone-capstone/ros2_ws/log

# ── 12. Shell setup ───────────────────────────────────────────────────
SHELL ["/bin/bash", "-c"]

RUN cat >> ~/.bashrc << 'BASHRC'
source /opt/ros/humble/setup.bash
source /opt/realsense_ros_ws/install/setup.bash
source /drone-capstone/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Our from-source libs must take precedence over anything in /usr/lib
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# Full clean rebuild
alias cb='cd /drone-capstone/ros2_ws && rm -rf build install log && \
    colcon build \
        --cmake-args \
            -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
            -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
            -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash'

# Incremental rebuild
alias b='cd /drone-capstone/ros2_ws && \
    colcon build \
        --cmake-args \
            -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
            -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
            -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash'

# PX4 SITL (GUI)
alias martin='cd /PX4-Autopilot && PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam_down NAV_DLL_ACT=0'

# PX4 SITL (headless)
alias jp='cd /PX4-Autopilot && PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam_down NAV_DLL_ACT=0 HEADLESS=1'

# Micro-XRCE-DDS-Agent over serial with auto-reconnect
# Usage: xrce_serial [device] [baud]   defaults: /dev/ttyTHS1 921600
xrce_serial() {
    local DEV="${1:-/dev/ttyTHS1}"
    local BAUD="${2:-921600}"
    echo "[xrce_serial] supervising ${DEV} @ ${BAUD} -- Ctrl-C to stop"
    while true; do
        while [ ! -e "${DEV}" ]; do
            echo "[xrce_serial] waiting for ${DEV}..."; sleep 2
        done
        echo "[xrce_serial] launching agent"
        MicroXRCEAgent serial -D "${DEV}" -b "${BAUD}" || true
        echo "[xrce_serial] agent exited, retrying in 1s"
        sleep 1
    done
}
export -f xrce_serial

alias mission='cd /drone-capstone/ros2_ws && source install/setup.bash && \
    ros2 run precision_land_apriltag precision_land_apriltag_node'

alias missionb='cd /drone-capstone/ros2_ws && \
    colcon build --packages-select precision_land_apriltag && \
    source install/setup.bash && \
    ros2 run precision_land_apriltag precision_land_apriltag_node'

alias viz='ros2 run rqt_image_view rqt_image_view --clear-config'

tmux
BASHRC

CMD ["bash"]
