#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "parser.h"
#include "protocol.h"
#include "comm_port.h"

#include "bea_sensors/msg/emergency.hpp"
#include "bea_sensors/msg/heartbeat.hpp"
#include "bea_sensors/msg/parameters.hpp"

namespace bea_sensors {

class Flatscan : public rclcpp::Node {
 public:
  Flatscan();
  ~Flatscan();

  void SpinOnce();

 private:
  bool Initialize();
  bool InitializeConfiguration(const Parameters& parameters);
  void HandleReceivedData(char* data, int length);
  void OnConnected();

 private:
  rclcpp::Time last_scan_stamp_;
  rclcpp::Time last_heartbeat_stamp_;
  rclcpp::Time last_emergency_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Publisher<bea_sensors::msg::Emergency>::SharedPtr emergency_publisher_;
  rclcpp::Publisher<bea_sensors::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr spin_timer_;

  CommPort<Flatscan> com_;
  Protocol protocol_;
  Parser parser_;
};

}  // namespace bea_sensors
