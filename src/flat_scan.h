#pragma once

#include <ros/ros.h>

#include "bea_sensors/Configure.h"
#include "protocol.h"
#include "serial_port.h"
#include "serial_port_impl.h"

namespace bea_sensors {

constexpr float kHighDensityResolution{0.18 + 0.09};
constexpr float kHighDensityRefreshPeriod{43 * 1e-3};
constexpr float kHighSpeedResolution{0.74 + 0.09};
constexpr float kHighSpeedRefreshPeriod{10.75 * 1e-3};

struct Parameters {
  uint8_t temperature = 1;
  uint8_t information = 0;
  uint8_t mode = 1;
  uint8_t optimization = 0;
  uint8_t counter = 1;
  uint8_t heartbeat_period = 5;
  uint8_t facet = 1;
  uint8_t averaging = 0;
  uint16_t number_of_spots = 400;
  uint16_t angle_first = 0;
  uint16_t angle_last = 10800;
};

class FlatScan {
 public:
  FlatScan(const ros::NodeHandle& nh);
  ~FlatScan();

 private:
  bool Initialize();
  bool HandleConfiguration(Configure::Request& req, Configure::Response& res);
  void HandleSetBaudrate(Configure::Request& req, Configure::Response& res);
  void HandleGetMeasurements(Configure::Request& req, Configure::Response& res);
  void HandleSetParameters(Configure::Request& req, Configure::Response& res);
  void HandleSetLed(Configure::Request& req, Configure::Response& res);
  void SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data);

  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void ParseMdiMessage(const uint8_t* data, const int& length);
  void ParseSendIdentityMessage(const uint8_t* data, const int& length);
  void ParseSendParametersMessage(const uint8_t* data, const int& length);
  void ParseHeartbeatMessage(const uint8_t* data, const int& length);
  void ParseEmergencyMessage(const uint8_t* data, const int& length);

 private:
  float resolution_ = 0., refresh_period_ = 0.;
  float first_angle_ = 0., last_angle_ = 0.;
  float min_range_ = 0., max_range_ = 0.;
  std::string frame_id_;

  ros::NodeHandle nh_;
  ros::Publisher laser_scan_publisher_;
  ros::Publisher emergency_publisher_;
  ros::Publisher heartbeat_publisher_;
  ros::ServiceServer configuration_server_;

  Parameters parameters_;
  SerialPort<FlatScan> com_;
  Protocol protocol_;
};

}  // namespace bea_sensors
