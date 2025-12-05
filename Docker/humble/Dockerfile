# Single-stage unified image for ROS2 Foxy + librealsense + RealSense ROS wrapper
FROM --platform=linux/arm64 ros:foxy

############################################################
# 1. Install ALL build + runtime dependencies
############################################################
RUN apt-get update && apt-get install -y \
    git \
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
    libssl1.1 \
    libgtk-3-dev \
    libgtk-3-0 \
    libglfw3-dev \
    libglfw3 \
    \
    # ROS2 dependencies for building
    python3-colcon-common-extensions \
    ros-foxy-diagnostic-updater \
    ros-foxy-sensor-msgs \
    ros-foxy-std-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-nav-msgs \
    ros-foxy-tf2 \
    ros-foxy-tf2-ros \
    \
    # Required for T265 realsense2_camera runtime
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-vision-msgs \
    ros-foxy-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*


############################################################
# 2. Build librealsense 2.53.1 (T265 last working version)
############################################################
RUN cd /tmp && \
    wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.53.1.tar.gz && \
    tar -xzf v2.53.1.tar.gz && \
    cd librealsense-2.53.1 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_EXAMPLES=true \
             -DBUILD_TOOLS=true \
             -DBUILD_GRAPHICAL_EXAMPLES=false \
             -DBUILD_PYTHON_BINDINGS=false \
             -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/librealsense-2.53.1 /tmp/v2.53.1.tar.gz


############################################################
# 3. Build the RealSense ROS2 wrapper
############################################################
WORKDIR /ros2_ws
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-legacy

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install --packages-select realsense2_camera_msgs realsense2_camera"

############################################################
# 4. Setup udev rules
############################################################
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b37", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/99-realsense-libusb.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/99-realsense-libusb.rules


############################################################
# 5. Create non-root ROS user
############################################################
RUN groupadd -r ros && \
    useradd -r -g ros -G dialout,plugdev,video -m -s /bin/bash -c "ROS user" ros && \
    chown -R ros:ros /ros2_ws

USER ros
WORKDIR /ros2_ws


############################################################
# 6. Add ROS environment setup
############################################################
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc


############################################################
# 7. Health check
############################################################
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD rs-enumerate-devices > /dev/null 2>&1 || exit 1


############################################################
# 8. Metadata + entrypoint
############################################################
LABEL maintainer="T265 ROS2 Docker Project" \
      description="Intel RealSense T265 tracking camera with ROS2 Foxy" \
      version="1.0" \
      ros.distro="foxy" \
      hardware.support="Intel RealSense T265" \
      librealsense.version="2.53.1"

############################################################
# 9. Install RViz
############################################################
USER root
RUN apt-get update && apt-get install -y \
    ros-foxy-rviz2 \
    && rm -rf /var/lib/apt/lists/*
USER ros

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
