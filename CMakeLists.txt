cmake_minimum_required(VERSION 3.10.2)

PROJECT(foxglove_bridge LANGUAGES CXX VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ASAN "Enable Address Sanitizer" OFF)

find_package(OpenSSL REQUIRED)
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
  set(DESIRED_WARNINGS "-Wall -Wextra -Wconversion -Wunreachable-code -Wuninitialized -pedantic-errors -Wold-style-cast -Wno-error=unused-variable -Wshadow -Wfloat-equal")
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

include_directories(foxglove_bridge_base/include)

# http://answers.ros.org/question/230877/optionally-build-a-package-with-catkin/
if(CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
  set(COMPILING_ROS1 1)

  message(STATUS "--------------------------------------------------")
  message(STATUS "foxglove_bridge is being built for ROS 1.")
  message(STATUS "--------------------------------------------------")

  find_package(catkin REQUIRED COMPONENTS roslib roscpp)

  catkin_package(
    INCLUDE_DIRS foxglove_bridge_base/include
    LIBRARIES foxglove_bridge_base
    CATKIN_DEPENDS roslib roscpp
    DEPENDS OpenSSL websocketpp
  )

  message(STATUS ${catkin_INCLUDE_DIRS})
  add_definitions(-DIS_ROS1)

  add_executable(foxglove_bridge ros1_foxglove_bridge/ros1_foxglove_bridge.cpp)
  target_include_directories(foxglove_bridge SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(foxglove_bridge OpenSSL::Crypto OpenSSL::SSL ${catkin_LIBRARIES})
elseif(DEFINED ENV{AMENT_PREFIX_PATH})
  set(COMPILING_ROS2 1)

  find_package(ament_cmake REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp REQUIRED)

  message(STATUS "--------------------------------------------------")
  message(STATUS "foxglove_bridge is being built for ROS 2.")
  message(STATUS "--------------------------------------------------")

  message(STATUS ${catkin_INCLUDE_DIRS})
  
  add_definitions(-DIS_ROS2)

  add_executable(foxglove_bridge ros2_foxglove_bridge/ros2_foxglove_bridge.cpp)
  target_include_directories(foxglove_bridge SYSTEM PRIVATE ${ament_index_cpp_INCLUDE_DIRS})
  ament_target_dependencies(foxglove_bridge rclcpp OpenSSL websocketpp)
  target_link_libraries(foxglove_bridge OpenSSL::Crypto OpenSSL::SSL)
else()
  message(FATAL_ERROR "No ROS 1 or ROS 2 build environment detected.")
endif()

#### INSTALL ###################################################################

if(COMPILING_ROS1)
    install(TARGETS foxglove_bridge
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
elseif(COMPILING_ROS2)
    install(TARGETS foxglove_bridge
      DESTINATION lib/${PROJECT_NAME})

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
endif()
