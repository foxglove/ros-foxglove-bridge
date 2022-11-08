FROM ros:noetic-ros-core

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  clang-format \
  curl \
  git \
  gnupg \
  libasio-dev \
  libssl-dev \
  libwebsocketpp-dev \
  lsb-release \
  nlohmann-json3-dev \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  && rm -rf /var/lib/apt/lists/*

# Authorize the ROS 2 GPG key and add the ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-galactic-ros-core \
  && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Create a ROS workspace
RUN mkdir -p /ros_ws/src/ros-foxglove-bridge

# Unset the ROS_DISTRO and add aliases to .bashrc
RUN echo $'\
unset ROS_DISTRO\n\
alias ros2_build_debug="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug"\n\
alias ros2_build_release="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"\n\
alias ros2_foxglove_bridge="source /ros_ws/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args --log-level debug"\n\
' >> ~/.bashrc
