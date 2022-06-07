cmake_minimum_required(VERSION 3.0)
project(bea_sensors VERSION 1.0.0)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(flat_scan
  src/demo.cc
  src/flat_scan.cc
  src/protocol.cc
)

add_dependencies(flat_scan
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(flat_scan
  ${catkin_LIBRARIES}
)
