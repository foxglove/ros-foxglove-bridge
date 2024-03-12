ARG ROS_DISTRIBUTION=humble
FROM ros:$ROS_DISTRIBUTION-ros-base

# Install clang and set as default compiler.
RUN apt-get update && apt-get install -y --no-install-recommends \
  clang \
  && rm -rf /var/lib/apt/lists/*

ENV CC=clang
ENV CXX=clang++

# Set environment and working directory
ENV ROS_WS /ros_ws
WORKDIR $ROS_WS

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  clang-15 \
  clangd-15 \
  clang-format-15 \
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

# Set clang*-15 as the default
RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-15 100
RUN update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-15 100
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-15 100

# Add package.xml so we can install package dependencies
COPY package.xml src/ros-foxglove-bridge/

# Install rosdep dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep update --include-eol-distros && rosdep install -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

ENV COLCON_DEFAULTS_FILE=/ros_ws/src/ros-foxglove-bridge/.devcontainer/colcon_defaults.yaml

SHELL ["/bin/bash", "-c"]

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

# Make the workspace directory writable by the non-root user
RUN chown -R $USERNAME:$USERNAME $ROS_WS

USER $USERNAME

# Add aliases to .bashrc
RUN echo $'\
alias ros2_build_debug="source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON"\n\
alias ros2_build_release="source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON"\n\
alias ros2_foxglove_bridge="source /ros_ws/install_ros2/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args --log-level debug --log-level rcl:=INFO"\n\
' >> ~/.bashrc
