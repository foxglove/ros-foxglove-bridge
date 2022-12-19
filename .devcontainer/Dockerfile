FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  clang-format \
  curl \
  git-lfs \
  lldb \
  openssh-client \
  ros-$ROS_DISTRO-foxglove-msgs \
  ros-$ROS_DISTRO-rosbag2-storage-mcap \
  strace \
  && rm -rf /var/lib/apt/lists/*

# Create a ROS workspace
RUN mkdir -p /ros_ws/src/ros-foxglove-bridge

COPY package.xml /ros_ws/src/ros-foxglove-bridge/package.xml

# Install rosdep dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep update && rosdep install -y \
      --from-paths /ros_ws/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Unset the ROS_DISTRO and add aliases to .bashrc
RUN echo \
  'alias ros2_build_debug="/ros_entrypoint.sh colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug"\n'\
  'alias ros2_build_release="/ros_entrypoint.sh colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"\n'\
  'alias ros2_foxglove_bridge="source /ros_ws/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args --log-level debug --log-level rcl:=INFO"\n'\
>> ~/.bashrc
