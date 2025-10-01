ROS1_DISTRIBUTIONS := melodic noetic

define generate_ros1_targets
.PHONY: $(1)
$(1):
	docker build -t foxglove_bridge_$(1) --pull -f Dockerfile.ros1 --build-arg ROS_DISTRIBUTION=$(1) .

.PHONY: $(1)-test
$(1)-test: $(1)
	docker run -t --rm foxglove_bridge_$(1) bash -c "catkin_make run_tests && catkin_test_results"

.PHONY: $(1)-boost-asio
$(1)-boost-asio:
	docker build -t foxglove_bridge_$(1)_boost_asio --pull -f Dockerfile.ros1 --build-arg ROS_DISTRIBUTION=$(1) --build-arg USE_ASIO_STANDALONE=OFF .

.PHONY: $(1)-test-boost-asio
$(1)-test-boost-asio: $(1)-boost-asio
	docker run -t --rm foxglove_bridge_$(1)_boost_asio bash -c "catkin_make run_tests && catkin_test_results"
endef

$(foreach distribution,$(ROS1_DISTRIBUTIONS),$(eval $(call generate_ros1_targets,$(strip $(distribution)))))


default: ros1

.PHONY: ros1
ros1:
	docker build -t foxglove_bridge_ros1 --pull -f Dockerfile.ros1 .

clean:
	docker rmi $(docker images --filter=reference="foxglove_bridge_*" -q)
