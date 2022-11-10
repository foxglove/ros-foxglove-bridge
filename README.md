# ros-foxglove-bridge
ROS 1 and ROS 2 C++ WebSocket bridge using ws-protocol

## Development

A VSCode container is provided with a dual ROS 1 and ROS 2 installation and
enough tools to build and run the bridge. Some bash aliases are defined:

```bash
# Enable the ROS 2 environment
source /opt/ros/galactic/setup.bash
# Build the bridge
$ ros2_build_debug
$ ros2_build_release
# Run the bridge
$ ros2_foxglove_bridge
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
