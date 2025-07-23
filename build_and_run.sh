. /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --event-handlers console_direct+ --cmake-args -DUSE_ASIO_STANDALONE=$USE_ASIO_STANDALONE \
  && . ./install/setup.sh \
  && ros2 run foxglove_bridge foxglove_bridge --ros-args -p topic_throttle_rates:="[1.0]" -p topic_throttle_patterns:="['.*actuators.*']"
