cmake_minimum_required(VERSION 3.0)
project(bea_sensors VERSION 1.0.0)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Emergency.msg
  Heartbeat.msg
  Parameters.msg
)

add_service_files(
  FILES
  Configure.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(flatscan
  src/demo.cc
  src/flatscan.cc
  src/protocol.cc
  src/parser.cc
)

add_dependencies(flatscan
	bea_sensors_generate_messages_cpp
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(flatscan
  ${catkin_LIBRARIES}
)
