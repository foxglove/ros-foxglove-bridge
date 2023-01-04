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

## Installation

The `foxglove_bridge` package is available for ROS 1 Melodic and Noetic, and ROS 2 Humble and Rolling. Earlier releases of ROS will not be supported due to API design and/or performance limitations. The package can be installed with the following command:

```bash
$ sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

## Running the bridge

To run the bridge node, it is recommended to use the provided launch file:

**ROS 1**

```bash
$ roslaunch foxglove_bridge foxglove_bridge.launch --screen
```

**ROS 2**

```bash
$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Configuration

Parameters are provided to configure the behavior of the bridge. These parameters must be set at initialization through a launch file or the command line, they cannot be modified at runtime.

 * __port__: The TCP port to bind the WebSocket server to. Must be a valid TCP port number, or 0 to use a random port. Defaults to `8765`.
 * __address__: The host address to bind the WebSocket server to. Defaults to `0.0.0.0`, listening on all interfaces by default. Change this to `127.0.0.1` to only accept connections from the local machine.
 * __tls__: If `true`, use Transport Layer Security (TLS) for encrypted communication. Defaults to `false`.
 * __certfile__: Path to the certificate to use for TLS. Required when __tls__ is set to `true`. Defaults to `""`.
 * __keyfile__: Path to the private key to use for TLS. Required when __tls__ is set to `true`. Defaults to `""`.
 * __topic_whitelist__: List of regular expressions ([ECMAScript grammar](https://en.cppreference.com/w/cpp/regex/ecmascript)) of whitelisted topic names. Defaults to `[".*"]`.
 * __send_buffer_limit__: Connection send buffer limit in bytes. Messages will be dropped when a connection's send buffer reaches this limit to avoid a queue of outdated messages building up. Defaults to `10000000` (10 MB).
 * (ROS 1) __max_update_ms__: The maximum number of milliseconds to wait in between polling `roscore` for new topics, services, or parameters. Defaults to `5000`.
 * (ROS 2) __num_threads__: The number of threads to use for the ROS node executor. This controls the number of subscriptions that can be processed in parallel. 0 means one thread per CPU core. Defaults to `0`.
 * (ROS 2) __max_qos_depth__: Maximum depth used for the QoS profile of subscriptions. Defaults to `10`.


## Clients

[Foxglove Studio](https://foxglove.dev/studio) connects to `foxglove_bridge` for live robotics visualization.

### Building from source

You can also try `foxglove_bridge` by building from source or running a pre-built Docker container.

#### Fetch source and install dependencies

```bash
cd <path/to/your/ros_ws>
git clone https://github.com/foxglove/ros-foxglove-bridge.git src/ros-foxglove-bridge
rosdep update
rosdep install -i --from-path src -y
```

#### ROS 1
```
catkin_make
source install/local_setup.bash
rosrun foxglove_bridge foxglove_bridge
```

#### ROS 2
```
colcon build
source install/local_setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

### Docker

#### ROS 1

```bash
docker run --rm -it -v /opt/ros:/opt/ros --net=host ghcr.io/foxglove/noetic-ros1-bridge
```

#### ROS 2

```bash
docker run --rm -it -v /opt/ros:/opt/ros --net=host ghcr.io/foxglove/humble-ros2-bridge
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
