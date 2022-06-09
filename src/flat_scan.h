#pragma once

#include <ros/ros.h>

#include "bea_sensors/Configure.h"
#include "protocol.h"
#include "serial_port.h"
#include "serial_port_impl.h"

namespace bea_sensors {

// constexpr uint16_t kDefaultLaserDataLength{400};

// struct LaserData {
//   uint16_t length = 0;
//   float* ranges = nullptr;
//   float* intensities = nullptr;

//   LaserData() : ranges(new float[kDefaultLaserDataLength]), intensities(new float[kDefaultLaserDataLength]) {}
//   LaserData(const uint16_t& l) : ranges(new float[l]), intensities(new float[l]) {}
//   LaserData(const LaserData& other) {
//     length = other.length;
//     ranges = new float[other.length];
//     intensities = new float[other.length];
//     std::copy(other.ranges, other.ranges + other.length, ranges);
//     std::copy(other.intensities, other.intensities + other.length, intensities);
//   }
//   ~LaserData() {
//     delete[] ranges;
//     delete[] intensities;
//   }

//   LaserData& operator=(const LaserData& other) {
//     if (this == &other) {
//       return *this;
//     }

//     length = other.length;

//     if (ranges) {
//       delete[] ranges;
//       ranges = nullptr;
//     }
//     ranges = new float[other.length];
//     std::copy(other.ranges, other.ranges + other.length, ranges);

//     if (intensities) {
//       delete[] intensities;
//       intensities = nullptr;
//     }
//     intensities = new float[other.length];
//     std::copy(other.intensities, other.intensities + other.length, intensities);

//     return *this;
//   }
// };

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

  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data);

 private:
  uint8_t information_ = 0;
  uint16_t number_of_spots_ = 0;
  float resolution_ = 0., refresh_period_ = 0.;
  float first_angle_ = 0., last_angle_ = 0.;
  float min_range_ = 0., max_range_ = 0.;
  std::string frame_id_;

  uint8_t* parameters_ = nullptr;

  ros::NodeHandle nh_;
  ros::Publisher laser_scan_publisher_;
  ros::ServiceServer configuration_server_;

  SerialPort<FlatScan> com_;
  Protocol protocol_;
};

}  // namespace bea_sensors
