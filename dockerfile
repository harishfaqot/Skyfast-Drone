FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/catkin_ws

# System + ROS build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    nano \
    python3-rosdep \
    python3-catkin-tools \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    libceres-dev \
    geographiclib-tools \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# MAVROS geoid data
RUN geographiclib-get-geoids egm96-5

# rosdep
RUN rosdep init || true
RUN rosdep update

CMD ["bash"]
