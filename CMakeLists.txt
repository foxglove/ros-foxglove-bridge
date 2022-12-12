cmake_minimum_required(VERSION 3.10.2)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
endif()
if(POLICY CMP0024)
  cmake_policy(SET CMP0024 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0024 NEW)
endif()

project(foxglove_bridge LANGUAGES CXX VERSION 0.2.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ASAN "Enable Address Sanitizer" OFF)

find_package(nlohmann_json QUIET)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(websocketpp REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(OPTIMIZATION_FLAGS "-DDEBUG")
  if(NOT MSVC)
    set(OPTIMIZATION_FLAGS "${OPTIMIZATION_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer")
  endif()
  message("-- Configuring debug build")
else()
  set(OPTIMIZATION_FLAGS "-DNDEBUG -O2")
  message("-- Configuring release build")
endif()

if(MSVC)
  set(DESIRED_WARNINGS "-WX")
else()
  set(DESIRED_WARNINGS "-Wall -Wextra -Wconversion -Wunreachable-code -Wuninitialized -pedantic-errors -Wold-style-cast -Wno-error=unused-variable -Wfloat-equal")
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(DESIRED_WARNINGS "${DESIRED_WARNINGS} -Wmost")
  endif()
  set(DESIRED_WARNINGS "${DESIRED_WARNINGS} -Werror")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPTIMIZATION_FLAGS} ${DESIRED_WARNINGS}")

IF(NOT WIN32 AND ENABLE_ASAN)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
  set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
endif()

# Detect big-endian architectures
include(TestBigEndian)
TEST_BIG_ENDIAN(ENDIAN)
if (ENDIAN)
  add_compile_definitions(ARCH_IS_BIG_ENDIAN=1)
endif()

# Build the foxglove_bridge_base library
add_library(foxglove_bridge_base SHARED
  foxglove_bridge_base/src/foxglove_bridge.cpp
  foxglove_bridge_base/src/test/test_client.cpp
)
target_include_directories(foxglove_bridge_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/foxglove_bridge_base/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(foxglove_bridge_base
  OpenSSL::Crypto
  OpenSSL::SSL
  ${CMAKE_THREAD_LIBS_INIT}
)
if(nlohmann_json_FOUND)
  target_link_libraries(foxglove_bridge_base nlohmann_json::nlohmann_json)
else()
  message(STATUS "nlohmann_json not found, will search at compile time")
endif()

message(STATUS "ROS_VERSION: " $ENV{ROS_VERSION})
message(STATUS "ROS_DISTRO: " $ENV{ROS_DISTRO})
message(STATUS "ROS_ROOT: " $ENV{ROS_ROOT})
if("$ENV{ROS_VERSION}" STREQUAL "1")
  # ROS 1
  if(CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    message(STATUS "Building with catkin")
    set(ROS_BUILD_TYPE "catkin")

    find_package(catkin REQUIRED COMPONENTS nodelet ros_babel_fish rosgraph_msgs roslib roscpp)
    find_package(Boost REQUIRED)

    catkin_package(
      INCLUDE_DIRS foxglove_bridge_base/include
      LIBRARIES foxglove_bridge_base foxglove_bridge_nodelet
      CATKIN_DEPENDS nodelet ros_babel_fish rosgraph_msgs roslib roscpp
      DEPENDS Boost
    )

    add_library(foxglove_bridge_nodelet ros1_foxglove_bridge/src/ros1_foxglove_bridge_nodelet.cpp)
    target_include_directories(foxglove_bridge_nodelet SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(foxglove_bridge_nodelet foxglove_bridge_base ${catkin_LIBRARIES})

    add_executable(foxglove_bridge ros1_foxglove_bridge/src/ros1_foxglove_bridge_node.cpp)
    target_include_directories(foxglove_bridge SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(foxglove_bridge ${catkin_LIBRARIES})
  else()
    message(FATAL_ERROR "Could not find catkin")
  endif()
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  # ROS 2
  if(DEFINED ENV{AMENT_PREFIX_PATH})
    message(STATUS "Building with ament_cmake")
    set(ROS_BUILD_TYPE "ament_cmake")

    find_package(ament_cmake REQUIRED)
    find_package(rosgraph_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_components REQUIRED)

    add_library(foxglove_bridge_component SHARED
      ros2_foxglove_bridge/src/message_definition_cache.cpp
      ros2_foxglove_bridge/src/ros2_foxglove_bridge.cpp
    )
    target_include_directories(foxglove_bridge_component
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/foxglove_bridge_base/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros2_foxglove_bridge/include>
        $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(foxglove_bridge_component rclcpp rclcpp_components rosgraph_msgs)
    target_link_libraries(foxglove_bridge_component foxglove_bridge_base)
    rclcpp_components_register_nodes(foxglove_bridge_component "foxglove_bridge::FoxgloveBridge")

    add_executable(foxglove_bridge
      ros2_foxglove_bridge/src/ros2_foxglove_bridge_node.cpp
    )
    target_include_directories(foxglove_bridge SYSTEM PRIVATE ${rclcpp_INCLUDE_DIRS})
    ament_target_dependencies(foxglove_bridge rclcpp rclcpp_components)
  else()
    message(FATAL_ERROR "Could not find ament_cmake")
  endif()
else()
  message(FATAL_ERROR "ROS_VERSION environment variable must be 1 or 2")
endif()

#### TESTS #####################################################################

if(ROS_BUILD_TYPE STREQUAL "catkin")
  if (CATKIN_ENABLE_TESTING)
    message(STATUS "Building tests with catkin")

    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
    if(NOT "$ENV{ROS_DISTRO}" STREQUAL "melodic")
      find_package(GTest REQUIRED)
    endif()
    find_package(rostest REQUIRED)

    catkin_add_gtest(version_test foxglove_bridge_base/tests/version_test.cpp)
    target_link_libraries(version_test foxglove_bridge_base)

    add_rostest_gtest(smoke_test ros1_foxglove_bridge/tests/smoke.test ros1_foxglove_bridge/tests/smoke_test.cpp)
    target_include_directories(smoke_test SYSTEM PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/foxglove_bridge_base/include>
      ${catkin_INCLUDE_DIRS}
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(smoke_test foxglove_bridge_base ${catkin_LIBRARIES})
  endif()
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
  if(BUILD_TESTING)
    message(STATUS "Building tests with ament_cmake")

    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(version_test foxglove_bridge_base/tests/version_test.cpp)
    target_link_libraries(version_test foxglove_bridge_base)

    ament_add_gtest(smoke_test ros2_foxglove_bridge/tests/smoke_test.cpp)
    ament_target_dependencies(smoke_test rclcpp rclcpp_components std_msgs)
    target_link_libraries(smoke_test foxglove_bridge_base)
  endif()
endif()

#### INSTALL ###################################################################

if(ROS_BUILD_TYPE STREQUAL "catkin")
    install(TARGETS foxglove_bridge
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(TARGETS foxglove_bridge_base foxglove_bridge_nodelet
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    install(FILES nodelets.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
    install(TARGETS foxglove_bridge
      DESTINATION lib/${PROJECT_NAME}
    )
    install(TARGETS foxglove_bridge_base foxglove_bridge_component
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    ament_export_libraries(foxglove_bridge_base foxglove_bridge_component)
    ament_package()
endif()
