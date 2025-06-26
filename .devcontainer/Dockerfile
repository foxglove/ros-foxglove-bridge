ARG ROS_DISTRIBUTION=humble
FROM ros:$ROS_DISTRIBUTION-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  clang \
  clang-format \
  clangd-14 \
  curl \
  gdb \
  git \
  git-lfs \
  gnupg \
  libasio-dev \
  libboost-all-dev \
  libssl-dev \
  libwebsocketpp-dev \
  nlohmann-json3-dev \
  strace \
  sudo \
  && rm -rf /var/lib/apt/lists/*

# Set clang as default compiler
ENV CC=clang
ENV CXX=clang++

# Configure clangd
RUN update-alternatives --install /usr/local/bin/clangd clangd $(which clangd-14) 1000

# Create a non-root user
# Taken from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user and add them to sudoers
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# Set environment and working directory
ENV ROS_WS=/ros_ws
WORKDIR $ROS_WS
RUN sudo mkdir -p $ROS_WS/src && sudo chown -R $USERNAME:$USERNAME $ROS_WS

# Add package.xml so we can install package dependencies
COPY --chown=$USERNAME:$USERNAME package.xml src/ros-foxglove-bridge/

# Update rosdep and install dependencies
RUN rosdep update --include-eol-distros
RUN sudo apt-get update && rosdep install -y \
      --from-paths \
        src \
      --ignore-src \
    && sudo rm -rf /var/lib/apt/lists/*

ENV COLCON_DEFAULTS_FILE=/$ROS_WS/src/ros-foxglove-bridge/.devcontainer/colcon_defaults.yaml

SHELL ["/bin/bash", "-c"]

# Add aliases to .bashrc
RUN echo $'\
alias ros2_build_debug="source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"\n\
alias ros2_build_release="source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"\n\
alias ros2_foxglove_bridge="source /ros_ws/install_ros2/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args --log-level debug --log-level rcl:=INFO"\n\
' >> ~/.bashrc
