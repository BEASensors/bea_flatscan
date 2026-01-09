cmake_minimum_required(VERSION 3.5)
project(bea_sensors VERSION 1.0.0)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/Emergency.msg"
  "msg/Heartbeat.msg"
  "msg/Parameters.msg"
)

set(SRV_FILES
  "srv/Configure.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

add_executable(flatscan
  src/demo.cc
  src/flatscan.cc
  src/protocol.cc
  src/parser.cc
)

ament_target_dependencies(flatscan
  rclcpp
  std_msgs
  sensor_msgs
  angles
)

target_include_directories(flatscan PRIVATE
  ${Boost_INCLUDE_DIRS}
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  target_link_libraries(flatscan
    ${cpp_typesupport_target}
  )
else()
  rosidl_target_interfaces(flatscan
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
endif()

install(TARGETS flatscan
  DESTINATION lib/${PROJECT_NAME}
)

# Install ROS 2 launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.rviz"
)

ament_package() 