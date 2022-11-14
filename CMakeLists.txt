cmake_minimum_required(VERSION 3.10.2)

project(foxglove_bridge LANGUAGES CXX VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_ASAN "Enable Address Sanitizer" OFF)

find_package(nlohmann_json REQUIRED CONFIG PATHS "/usr/lib/cmake/")
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
)
target_include_directories(foxglove_bridge_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/foxglove_bridge_base/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(foxglove_bridge_base
  nlohmann_json
  OpenSSL::Crypto
  OpenSSL::SSL
  ${CMAKE_THREAD_LIBS_INIT}
)

message(STATUS "ROS_VERSION: " $ENV{ROS_VERSION})
if("$ENV{ROS_VERSION}" STREQUAL "1")
  # ROS 1
  if(CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    message(STATUS "Building with catkin")
    set(ROS_BUILD_TYPE "catkin")

    find_package(catkin REQUIRED COMPONENTS nodelet ros_type_introspection roslib roscpp rospack)
    find_package(Boost REQUIRED)

    catkin_package(
      INCLUDE_DIRS foxglove_bridge_base/include ros1_foxglove_bridge/include
      LIBRARIES foxglove_bridge_base foxglove_bridge_nodelet message_parser
      CATKIN_DEPENDS nodelet ros_type_introspection roslib roscpp rospack
      DEPENDS Boost
    )

    add_library(message_parser ros1_foxglove_bridge/src/msg_parser.cpp)
    target_include_directories(message_parser SYSTEM PRIVATE ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ros1_foxglove_bridge/include)
    target_link_libraries(message_parser ${catkin_LIBRARIES})

    add_executable(get_full_message_definition ros1_foxglove_bridge/src/msg_parser_cli.cpp)
    target_include_directories(get_full_message_definition SYSTEM PRIVATE ros1_foxglove_bridge/include)
    target_link_libraries(get_full_message_definition message_parser)

    add_library(foxglove_bridge_nodelet ros1_foxglove_bridge/src/ros1_foxglove_bridge_nodelet.cpp)
    target_include_directories(foxglove_bridge_nodelet SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} ros1_foxglove_bridge/include)
    target_link_libraries(foxglove_bridge_nodelet foxglove_bridge_base message_parser ${catkin_LIBRARIES})

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
    find_package(rclcpp REQUIRED)

    add_executable(foxglove_bridge
      ros2_foxglove_bridge/src/message_definition_cache.cpp
      ros2_foxglove_bridge/src/ros2_foxglove_bridge.cpp
    )
    target_include_directories(foxglove_bridge SYSTEM PRIVATE ${rclcpp_INCLUDE_DIRS})
    target_link_libraries(foxglove_bridge foxglove_bridge_base ${rclcpp_LIBRARIES})
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

    find_package(catkin REQUIRED COMPONENTS rostest)
    find_package(GTest REQUIRED)

    catkin_add_gtest(version_test foxglove_bridge_base/tests/version_test.cpp)
    target_link_libraries(version_test foxglove_bridge_base GTest::GTest GTest::Main)

    add_rostest(ros1_foxglove_bridge/tests/smoke.test)

    catkin_add_gtest(message_parser_test ros1_foxglove_bridge/tests/msg_parser_test.cpp)
    target_include_directories(message_parser_test SYSTEM PRIVATE ros1_foxglove_bridge/include)
    target_link_libraries(message_parser_test message_parser GTest::GTest GTest::Main)
  endif()
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
  if(BUILD_TESTING)
    message(STATUS "Building tests with ament_cmake")

    find_package(ament_lint_auto REQUIRED)
    find_package(GTest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(version_test foxglove_bridge_base/tests/version_test.cpp)
    target_link_libraries(version_test foxglove_bridge_base GTest::GTest GTest::Main)
  endif()
endif()

#### INSTALL ###################################################################

if(ROS_BUILD_TYPE STREQUAL "catkin")
    install(TARGETS foxglove_bridge get_full_message_definition
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(TARGETS foxglove_bridge_base foxglove_bridge_nodelet message_parser
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    install(DIRECTORY ros1_foxglove_bridge/include/foxglove_bridge
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
    install(FILES nodelets.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
    install(TARGETS foxglove_bridge
      DESTINATION lib/${PROJECT_NAME}
    )
    install(TARGETS foxglove_bridge_base
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    ament_export_libraries(foxglove_bridge_base)
    ament_package()
endif()
