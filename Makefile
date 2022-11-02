default: ros2-image

.PHONY: ros1-image
ros1-image:
	docker build -t foxglove_bridge_ros1 -f Dockerfile.ros1 .

.PHONY: ros2-image
ros2-image:
	docker build -t foxglove_bridge_ros2 -f Dockerfile.ros2 .

.PHONY: ros2-dev-image
ros2-dev-image:
	docker build -t foxglove_bridge_ros2dev -f .devcontainer/Dockerfile.ros2 .

clean:
	docker rmi -f foxglove_bridge_ros1
	docker rmi -f foxglove_bridge_ros2
	docker rmi -f foxglove_bridge_ros2dev

.PHONY: format-check
format-check: ros2-dev-image
	docker run -t --rm -v $(CURDIR):/src foxglove_bridge_ros2dev python3 /src/scripts/format.py /src
