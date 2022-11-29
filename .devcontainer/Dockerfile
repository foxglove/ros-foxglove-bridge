FROM ros:noetic-ros-core

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  clang-format \
  curl \
  git \
  git-lfs \
  gnupg \
  lldb \
  lsb-release \
  openssh-client \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  strace \
  && rm -rf /var/lib/apt/lists/*

# Authorize the ROS 2 GPG key and add the ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-galactic-foxglove-msgs \
  ros-galactic-ros-core \
  ros-galactic-rosbag2 \
  ros-galactic-rosbag2-storage-mcap \
  ros-galactic-tf2-msgs \
  && rm -rf /var/lib/apt/lists/*

# Create a ROS workspace
RUN mkdir -p /ros_ws/src/ros-foxglove-bridge

COPY package.xml /ros_ws/src/ros-foxglove-bridge/package.xml

# Initialize rosdep
RUN rosdep init && rosdep update

# Install rosdep dependencies for ROS 1
RUN . /opt/ros/noetic/setup.sh && \
    apt-get update && rosdep update && rosdep install -y \
      --from-paths /ros_ws/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Install rosdep dependencies for ROS 2
RUN . /opt/ros/galactic/setup.sh && \
    apt-get update && rosdep update && rosdep install -y \
      --from-paths /ros_ws/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Unset the ROS_DISTRO and add aliases to .bashrc
RUN echo $'\
unset ROS_DISTRO\n\
alias ros2_build_debug="colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug"\n\
alias ros2_build_release="colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"\n\
alias ros2_foxglove_bridge="source /ros_ws/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args --log-level debug --log-level rcl:=INFO"\n\
' >> ~/.bashrc
