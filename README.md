foxglove_bridge
===============

[![ROS Melodic version](https://img.shields.io/ros/v/melodic/foxglove_bridge)](https://index.ros.org/p/foxglove_bridge/github-foxglove-ros-foxglove-bridge/#melodic)
[![ROS Noetic version](https://img.shields.io/ros/v/noetic/foxglove_bridge)](https://index.ros.org/p/foxglove_bridge/github-foxglove-ros-foxglove-bridge/#noetic)
[![ROS Galactic version](https://img.shields.io/ros/v/galactic/foxglove_bridge)](https://index.ros.org/p/foxglove_bridge/github-foxglove-ros-foxglove-bridge/#galactic)
[![ROS Humble version](https://img.shields.io/ros/v/humble/foxglove_bridge)](https://index.ros.org/p/foxglove_bridge/github-foxglove-ros-foxglove-bridge/#humble)
[![ROS Rolling version](https://img.shields.io/ros/v/rolling/foxglove_bridge)](https://index.ros.org/p/foxglove_bridge/github-foxglove-ros-foxglove-bridge/#rolling)

High performance ROS 1 and ROS 2 WebSocket bridge using the Foxglove WebSocket protocol, written in C++.

## Motivation

Live debugging of ROS systems has traditionally relied on running ROS tooling such as rviz. This requires either a GUI and connected peripherals on the robot, or replicating the same ROS environment on a network-connected development machine including the same version of ROS, all custom message definitions, etc. To overcome this limitation and allow remote debugging from web tooling or non-ROS systems, rosbridge was developed. However, rosbridge suffers from performance problems with high frequency topics and/or large messages, and the protocol does not support full visibility into ROS systems such as interacting with parameters or seeing the full graph of publishers and subscribers.

The `foxglove_bridge` uses the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol), a similar protocol to rosbridge but with the ability to support additional schema formats such as ROS 2 `.msg` and ROS 2 `.idl`, parameters, graph introspection, and non-ROS systems. The bridge is written in C++ and designed for high performance with low overhead to minimize the impact to your robot stack.

## Installation and setup

For information on how to install and configure your `foxglove_bridge`, check out the full docs on the [Foxglove website](https://foxglove.dev/docs/studio/connection/using-foxglove-bridge).

## Build from source

To build `foxglove_bridge` from source, you must first fetch your source and install dependencies:

```bash
$ cd <path/to/your/ros_ws>
$ git clone https://github.com/foxglove/ros-foxglove-bridge.git src/ros-foxglove-bridge
$ rosdep update
$ rosdep install -i --from-path src -y
```

### ROS 1

```bash
$ catkin_make
$ source install/local_setup.bash
$ rosrun foxglove_bridge foxglove_bridge
```

### ROS 2

```bash
$ colcon build
$ source install/local_setup.bash
$ ros2 run foxglove_bridge foxglove_bridge
```

## Clients

[Foxglove Studio](https://foxglove.dev/studio) connects to foxglove_bridge for live robotics visualization.

## Development

A VSCode container is provided with a dual ROS 1 and ROS 2 installation and
enough tools to build and run the bridge. Some bash aliases are defined to simplify the common workflow. Here's an example of building and running the ROS 2 node:

```bash
source /opt/ros/galactic/setup.bash
ros2_build_debug  # or ros2_build_release
ros2_foxglove_bridge
```

To test the bridge with example data, open another terminal and download the test `.mcap` files:

```bash
./download_test_data.sh
```

Then start playback:

```bash
source /opt/ros/galactic/setup.bash
ros2 bag play -l --clock 100 -s mcap data/nuScenes-v1.0-mini-scene-0061-ros2.mcap
```

## License
`foxglove_bridge` is released with a MIT license. For full terms and conditions, see the [LICENSE](LICENSE) file.
