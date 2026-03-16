FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV Torch_DIR=/root/libs/libtorch
SHELL ["/bin/bash", "-c"]

# basic ubuntu tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    dirmngr \
    lsb-release \
    tzdata \
    git \
    wget \
    unzip \
    vim \
    build-essential \
    cmake \
    python3-pip \
    liblcm-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# for ROS 2 apt source
RUN curl -L -s -o /tmp/ros2-apt-source.deb \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb \
    && apt-get update \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 + develop tools + Gazebo + Nav2 + SLAM + control
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \ 
    ros-humble-joint-state-publisher-gui \
    libgazebo-dev \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-control-toolbox \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-teleop-twist-keyboard \
    ros-humble-octomap-ros \
    ros-humble-octomap-rviz-plugins \
    ros-humble-velodyne-laserscan \
    && rm -rf /var/lib/apt/lists/*

# rosdep initialization
RUN rosdep init || true
RUN rosdep update || true

# LibTorch(C++)
RUN mkdir -p /root/libs && cd /root/libs \
    && wget -O libtorch.zip https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip \
    && unzip libtorch.zip \
    && rm libtorch.zip

# bashrc setting
RUN echo 'export Torch_DIR=/root/libs/libtorch' >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# workspace
RUN mkdir -p /home/test_ws/src
WORKDIR /home/test_ws

CMD ["/bin/bash"]