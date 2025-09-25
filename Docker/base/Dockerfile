FROM ros:humble-perception-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    tmux \
    vim \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-desktop \
    ros-humble-librealsense2* \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt \
    && rm -f /usr/local/bin/cmake && rm -rf /root/.cache/pip && rm requirements.txt

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

RUN git clone --recursive https://github.com/mcgill-robotics/drone-capstone.git /drone-capstone

WORKDIR /drone-capstone/ros2_ws

RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build"

RUN echo "source /drone-capstone/ros2_ws/install/setup.bash" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
RUN echo "tmux" >> ~/.bashrc
