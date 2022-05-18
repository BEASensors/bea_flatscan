cmake_minimum_required(VERSION 2.8.3)
project(bea_sensors)

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

add_executable(bea_sensor_driver
  src/main.cc
  src/bea_sensor_driver.cc
  src/protocol.cc
)

add_dependencies(bea_sensor_driver
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(bea_sensor_driver
  ${catkin_LIBRARIES}
)
