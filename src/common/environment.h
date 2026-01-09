#pragma once

#ifdef ROS1_ENV
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>

#include "bea_sensors/Configure.h"
#include "bea_sensors/Emergency.h"
#include "bea_sensors/Heartbeat.h"
#include "bea_sensors/Parameters.h"

#define LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)

namespace bea_sensors {

typedef sensor_msgs::LaserScan LaserScan;

inline ros::Time Stamp() { return ros::Time::now(); }

}  // namespace bea_sensors
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "bea_sensors/msg/emergency.hpp"
#include "bea_sensors/msg/heartbeat.hpp"
#include "bea_sensors/msg/parameters.hpp"
#include "bea_sensors/srv/configure.hpp"

#define LOG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define LOG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), __VA_ARGS__)

namespace bea_sensors {

typedef sensor_msgs::msg::LaserScan LaserScan;
typedef msg::Emergency Emergency;
typedef msg::Heartbeat Heartbeat;
typedef msg::Parameters Parameters;

inline const rclcpp::Time Stamp() { return rclcpp::Clock(RCL_ROS_TIME).now(); }

}  // namespace bea_sensors
#endif