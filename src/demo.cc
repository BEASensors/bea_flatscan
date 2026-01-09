#include <rclcpp/rclcpp.hpp>

#include "flatscan.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bea_sensors::Flatscan>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
