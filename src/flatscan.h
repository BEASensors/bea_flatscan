#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "parser.h"
#include "protocol.h"
#include "comm_port.h"

namespace bea_sensors {

class Flatscan {
 public:
  Flatscan(const ros::NodeHandle& nh);
  ~Flatscan();

  void SpinOnce();

 private:
  bool Initialize();
  bool InitializeConfiguration(const Parameters& parameters);
  bool HandleConfiguration(Configure::Request& req, Configure::Response& res);
  void SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data);
  void HandleReceivedData(char* data, int length);

 private:
  bool message_sent_ = false;
  std::mutex mutex_;

  ros::Time last_scan_stamp_ = ros::Time::now();
  ros::Time last_heartbeat_stamp_ = ros::Time::now();
  ros::Time last_emergency_stamp_ = ros::Time::now();

  ros::NodeHandle nh_;
  ros::Publisher laser_scan_publisher_;
  ros::Publisher emergency_publisher_;
  ros::Publisher heartbeat_publisher_;
  ros::ServiceServer configuration_server_;

  CommPort<Flatscan> com_;
  Protocol protocol_;
  Parser parser_;
};

}  // namespace bea_sensors
