foxglove_bridge
===============

High performance ROS 1 WebSocket bridge using the Foxglove WebSocket protocol, written in C++.
If you are looking for the ROS 2 WebSocket bridge, please go to https://github.com/foxglove/foxglove-sdk.

## Motivation

Live debugging of ROS systems has traditionally relied on running ROS tooling such as rviz. This requires either a GUI and connected peripherals on the robot, or replicating the same ROS environment on a network-connected development machine including the same version of ROS, all custom message definitions, etc. To overcome this limitation and allow remote debugging from web tooling or non-ROS systems, rosbridge was developed. However, rosbridge suffers from performance problems with high frequency topics and/or large messages, and the protocol does not support full visibility into ROS systems such as interacting with parameters or seeing the full graph of publishers and subscribers.

The `foxglove_bridge` uses the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol), a similar protocol to rosbridge but with the ability to support additional schema formats, parameters, graph introspection, and non-ROS systems. The bridge is written in C++ and designed for high performance with low overhead to minimize the impact to your robot stack.

## Installation

**Note**: While binary packages are available for ROS 1, all ROS 1 distributions are End-of-Life. That means that new binary packages of `foxglove_bridge` cannot be released into those distributions, and the versions are quite outdated. It is highly recommended to [build foxglove_bridge from source](#building-from-source) for the latest bug fixes. For similar reasons, this `foxglove_bridge` package for ROS 1 is in maintenance mode and only bug fixes will be applied.

## Running the bridge

To run the bridge node, it is recommended to use the provided launch file:

**ROS 1**

```bash
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
```

```xml
<launch>
  <!-- Including in another launch file -->
  <include file="$(find foxglove_bridge)/launch/foxglove_bridge.launch">
    <arg name="port" value="8765" />
    <!-- ... other arguments ... -->
  </include>
</launch>
```

### Configuration

Parameters are provided to configure the behavior of the bridge. These parameters must be set at initialization through a launch file or the command line, they cannot be modified at runtime.

 * __port__: The TCP port to bind the WebSocket server to. Must be a valid TCP port number, or 0 to use a random port. Defaults to `8765`.
 * __address__: The host address to bind the WebSocket server to. Defaults to `0.0.0.0`, listening on all interfaces by default. Change this to `127.0.0.1` (or `::1` for IPv6) to only accept connections from the local machine.
 * __tls__: If `true`, use Transport Layer Security (TLS) for encrypted communication. Defaults to `false`.
 * __certfile__: Path to the certificate to use for TLS. Required when __tls__ is set to `true`. Defaults to `""`.
 * __keyfile__: Path to the private key to use for TLS. Required when __tls__ is set to `true`. Defaults to `""`.
 * __topic_whitelist__: List of regular expressions ([ECMAScript grammar](https://en.cppreference.com/w/cpp/regex/ecmascript)) of whitelisted topic names. Defaults to `[".*"]`.
 * __service_whitelist__: List of regular expressions ([ECMAScript grammar](https://en.cppreference.com/w/cpp/regex/ecmascript)) of whitelisted service names. Defaults to `[".*"]`.
 * __param_whitelist__: List of regular expressions ([ECMAScript grammar](https://en.cppreference.com/w/cpp/regex/ecmascript)) of whitelisted parameter names. Defaults to `[".*"]`.
 * __client_topic_whitelist__: List of regular expressions ([ECMAScript grammar](https://en.cppreference.com/w/cpp/regex/ecmascript)) of whitelisted client-published topic names. Defaults to `[".*"]`.
 * __send_buffer_limit__: Connection send buffer limit in bytes. Messages will be dropped when a connection's send buffer reaches this limit to avoid a queue of outdated messages building up. Defaults to `10000000` (10 MB).
 * __use_compression__: Use websocket compression (permessage-deflate). It is recommended to leave this turned off as it increases CPU usage and per-message compression often yields low compression ratios for robotics data. Defaults to `false`.
 * __capabilities__: List of supported [server capabilities](https://github.com/foxglove/ws-protocol/blob/main/docs/spec.md). Defaults to `[clientPublish,parameters,parametersSubscribe,services,connectionGraph,assets]`.
 * __asset_uri_allowlist__: List of regular expressions ([ECMAScript grammar](https://en.cppreference.com/w/cpp/regex/ecmascript)) of allowed asset URIs. Uses the [resource_retriever](https://index.ros.org/p/resource_retriever/github-ros-resource_retriever) to resolve `package://`, `file://` or `http(s)://` URIs. Note that this list should be carefully configured such that no confidential files are accidentally exposed over the websocket connection. As an extra security measure, URIs containing two consecutive dots (`..`) are disallowed as they could be used to construct URIs that would allow retrieval of confidential files if the allowlist is not configured strict enough (e.g. `package://<pkg_name>/../../../secret.txt`). Defaults to `["^package://(?:[-\w%]+/)*[-\w%]+\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"]`.
 * __max_update_ms__: The maximum number of milliseconds to wait in between polling `roscore` for new topics, services, or parameters. Defaults to `5000`.
 * __service_type_retrieval_timeout_ms__: Max number of milliseconds for retrieving a services type information. Defaults to `250`.

## Building from source

### Fetch source, install dependencies, and build for ROS 1

```bash
source /opt/ros/noetic/setup.bash
mkdir -p foxglove_bridge_ws/src
cd foxglove_bridge_ws
git clone https://github.com/foxglove/ros-foxglove-bridge.git src/ros-foxglove-bridge
rosdep update
rosdep install --ignore-src --default-yes --from-path src
catkin_make install
source install/setup.bash
```

## Clients

[Foxglove](https://foxglove.dev/) connects to foxglove_bridge for live robotics visualization.

## License
`foxglove_bridge` is released with a MIT license. For full terms and conditions, see the [LICENSE](LICENSE) file.
