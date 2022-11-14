default: ros2

.PHONY: ros1
ros1:
	docker build -t foxglove_bridge_ros1 -f Dockerfile.ros1 .

.PHONY: ros2
ros2:
	docker build -t foxglove_bridge_ros2 -f Dockerfile.ros2 .

.PHONY: melodic
melodic:
	docker build -t foxglove_bridge_melodic -f Dockerfile.ros1 --build-arg ROS_DISTRIBUTION=melodic .

.PHONY: noetic
noetic:
	docker build -t foxglove_bridge_noetic -f Dockerfile.ros1 --build-arg ROS_DISTRIBUTION=noetic .

.PHONY: galactic
galactic:
	docker build -t foxglove_bridge_galactic -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=galactic .

.PHONY: humble
humble:
	docker build -t foxglove_bridge_humble -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=humble .

.PHONY: rolling
rolling:
	docker build -t foxglove_bridge_rolling -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=rolling .

.PHONY: rosdev
rosdev:
	docker build -t foxglove_bridge_rosdev -f .devcontainer/Dockerfile .

clean:
	docker rmi -f foxglove_bridge_ros1
	docker rmi -f foxglove_bridge_ros2
	docker rmi -f foxglove_bridge_melodic
	docker rmi -f foxglove_bridge_noetic
	docker rmi -f foxglove_bridge_galactic
	docker rmi -f foxglove_bridge_humble
	docker rmi -f foxglove_bridge_rolling
	docker rmi -f foxglove_bridge_rosdev

.PHONY: melodic-test
melodic-test: melodic
	docker run -t --rm foxglove_bridge_melodic catkin_make run_tests

.PHONY: noetic-test
noetic-test: noetic
	docker run -t --rm foxglove_bridge_noetic catkin_make run_tests

.PHONY: galactic-test
galactic-test: galactic
	docker run -t --rm foxglove_bridge_galactic colcon test --event-handlers console_cohesion+

.PHONY: humble-test
humble-test: humble
	docker run -t --rm foxglove_bridge_humble colcon test --event-handlers console_cohesion+

.PHONY: rolling-test
rolling-test: rolling
	docker run -t --rm foxglove_bridge_rolling colcon test --event-handlers console_cohesion+

.PHONY: lint
lint: rosdev
	docker run -t --rm -v $(CURDIR):/src foxglove_bridge_rosdev python3 /src/scripts/format.py /src
