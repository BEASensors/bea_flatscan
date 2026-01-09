#include "flatscan.h"

#include <angles/angles.h>

namespace bea_sensors {

Flatscan::Flatscan() : rclcpp::Node("bea_sensor_driver") {
  last_scan_stamp_ = this->now();
  last_heartbeat_stamp_ = this->now();
  last_emergency_stamp_ = this->now();
  Initialize();
  // 30 Hz timer to call SpinOnce
  spin_timer_ = this->create_wall_timer(std::chrono::milliseconds(33), [this]() { this->SpinOnce(); });
}

Flatscan::~Flatscan() { com_.Close(); }

void Flatscan::SpinOnce() {
  sensor_msgs::msg::LaserScan laser_scan = parser_.laser_scan();
  laser_scan.header.stamp = this->now();
  const rclcpp::Time current_scan_stamp{laser_scan.header.stamp};
  if (
      !laser_scan.header.frame_id.empty() &&
      ( !laser_scan.ranges.empty() || !laser_scan.intensities.empty() ) &&
      laser_scan_publisher_ &&
      laser_scan_publisher_->get_subscription_count() > 0 &&
      last_scan_stamp_ < current_scan_stamp) {
    laser_scan_publisher_->publish(laser_scan);
    last_scan_stamp_ = current_scan_stamp;
  }

  bea_sensors::msg::Heartbeat heartbeat = parser_.heartbeat();
  heartbeat.header.stamp = this->now();
  const rclcpp::Time current_heartbeat_stamp{heartbeat.header.stamp};
  if (heartbeat_publisher_ && heartbeat_publisher_->get_subscription_count() > 0 && last_heartbeat_stamp_ < current_heartbeat_stamp) {
    heartbeat_publisher_->publish(heartbeat);
    last_heartbeat_stamp_ = current_heartbeat_stamp;
  }

  bea_sensors::msg::Emergency emergency = parser_.emergency();
  emergency.header.stamp = this->now();
  const rclcpp::Time current_emergency_stamp{emergency.header.stamp};
  if (emergency_publisher_ && emergency_publisher_->get_subscription_count() > 0 && last_emergency_stamp_ < current_emergency_stamp) {
    emergency_publisher_->publish(emergency);
    last_emergency_stamp_ = current_emergency_stamp;
  }
}

bool Flatscan::Initialize() {
  // Parameters
  std::string type = this->declare_parameter<std::string>("communication", "serial");
  std::string comm_arg_1;
  int comm_arg_2;
  if (type == "serial") {
    comm_arg_1 = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    comm_arg_2 = this->declare_parameter<int>("baudrate", 921600);
  } else if (type == "ethernet") {
    comm_arg_1 = this->declare_parameter<std::string>("ip", "192.168.1.199");
    comm_arg_2 = this->declare_parameter<int>("port", 20108);
  }

  std::string scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
  std::string heartbeat_topic = this->declare_parameter<std::string>("heartbeat_topic", "/heartbeat");
  std::string emergency_topic = this->declare_parameter<std::string>("emergency_topic", "/emergency");

  Parameters parameters;
  parameters.header.frame_id = this->declare_parameter<std::string>("scan_frame_id", "laser_link");
  parameters.range_min = this->declare_parameter<double>("min_range", 0.0);
  parameters.range_max = this->declare_parameter<double>("max_range", 8.0);

  int temp;
  temp = this->declare_parameter<int>("enable_temperature", 1);
  parameters.temperature = static_cast<uint8_t>(temp);
  temp = this->declare_parameter<int>("information_in_mdi", 2);
  parameters.information = static_cast<uint8_t>(temp);
  std::string detection_field_mode = this->declare_parameter<std::string>("detection_field_mode", std::string("HD"));
  if (detection_field_mode == "HD") {
    RCLCPP_INFO(this->get_logger(), "Flatscan works in HD mode, loading HD configurations");
    parameters.mode = 1;
    parameters.number_of_spots = 400;
  } else if (detection_field_mode == "HS") {
    RCLCPP_INFO(this->get_logger(), "Flatscan works in HS mode, loading HS configurations");
    parameters.mode = 0;
    parameters.number_of_spots = 100;
  } else {
    RCLCPP_WARN(this->get_logger(), "Flatscan works in %s(UNKNOWN) mode, loading HD configurations by default", detection_field_mode.c_str());
    parameters.mode = 1;
    parameters.number_of_spots = 400;
  }
  temp = this->declare_parameter<int>("optimization", 0);
  parameters.optimization = static_cast<uint8_t>(temp);
  double angle_first = this->declare_parameter<double>("angle_first", 0.0);
  double angle_last = this->declare_parameter<double>("angle_last", 108.0);
  parameters.angle_first = static_cast<uint16_t>(angle_first * 1e2);
  parameters.angle_last = static_cast<uint16_t>(angle_last * 1e2);
  temp = this->declare_parameter<int>("enable_counter", 1);
  parameters.counter = static_cast<uint8_t>(temp);
  temp = this->declare_parameter<int>("heartbeat_period", 5);
  parameters.heartbeat_period = static_cast<uint8_t>(temp);
  temp = this->declare_parameter<int>("enable_facet", 1);
  parameters.facet = static_cast<uint8_t>(temp);
  temp = this->declare_parameter<int>("averaging_setting", 0);
  parameters.averaging = static_cast<uint8_t>(temp);

  // 强制固化为固定配置（无视设备返回），确保与需求一致
  parameters.temperature = 1;
  parameters.information = 2;
  parameters.mode = 1;                // HD
  parameters.optimization = 0;
  parameters.number_of_spots = 400;
  parameters.angle_first = static_cast<uint16_t>(0.0 * 1e2);
  parameters.angle_last  = static_cast<uint16_t>(108.0 * 1e2);
  parameters.counter = 1;
  parameters.heartbeat_period = 5;
  parameters.facet = 1;
  parameters.averaging = 0;

  laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
  heartbeat_publisher_ = this->create_publisher<bea_sensors::msg::Heartbeat>(heartbeat_topic, 10);
  emergency_publisher_ = this->create_publisher<bea_sensors::msg::Emergency>(emergency_topic, 10);

  com_.RegisterCallback(this, &Flatscan::HandleReceivedData);
  // Register on-connected to re-send configuration on link restore
  com_.RegisterConnectedCallback(this, &Flatscan::OnConnected);
  com_.Connect(type, comm_arg_1, comm_arg_2);

  InitializeConfiguration(parameters);
  return true;
}

bool Flatscan::InitializeConfiguration(const Parameters& parameters) {
  parser_.Initialize(parameters);
  return true;
}

void Flatscan::HandleReceivedData(char* data, int length) {
  for (int i = 0; i < length; ++i) {
    if (protocol_.InsertByte((uint8_t)data[i]) < 0) {
      continue;
    }

    DataFrame frame;
    if (!protocol_.GetLatestDataFrame(frame)) {
      RCLCPP_ERROR(this->get_logger(), "Get received frame error");
      continue;
    }

    if (!parser_.ParseDataFrame(frame)) {
      RCLCPP_ERROR(this->get_logger(), "Parse frame failed");
      continue;
    }
  }
}

void Flatscan::OnConnected() {
  // Re-send parameters upon link restoration to re-initialize the radar
  RCLCPP_INFO(this->get_logger(), "Link restored, re-sending parameters");
  InitializeConfiguration(parser_.parameters());
}

}  // namespace bea_sensors
