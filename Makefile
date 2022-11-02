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

.PHONY: foxy
foxy:
	docker build -t foxglove_bridge_foxy -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=foxy .

.PHONY: galactic
galactic:
	docker build -t foxglove_bridge_galactic -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=galactic .

.PHONY: humble
humble:
	docker build -t foxglove_bridge_humble -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=humble .

.PHONY: rolling
rolling:
	docker build -t foxglove_bridge_rolling -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=rolling .

.PHONY: ros2dev
ros2dev:
	docker build -t foxglove_bridge_ros2dev -f .devcontainer/Dockerfile.ros2 .

clean:
	docker rmi -f foxglove_bridge_ros1
	docker rmi -f foxglove_bridge_ros2
	docker rmi -f foxglove_bridge_melodic
	docker rmi -f foxglove_bridge_noetic
	docker rmi -f foxglove_bridge_foxy
	docker rmi -f foxglove_bridge_galactic
	docker rmi -f foxglove_bridge_humble
	docker rmi -f foxglove_bridge_rolling
	docker rmi -f foxglove_bridge_ros2dev

.PHONY: lint
lint: ros2dev
	docker run -t --rm -v $(CURDIR):/src foxglove_bridge_ros2dev python3 /src/scripts/format.py /src
