#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "bea_sensors/SetParameter.h"
#include "protocol.h"
#include "serial_port.h"
#include "serial_port_impl.h"

namespace bea_sensors {

constexpr uint16_t kDefaultLaserDataLength{400};

struct LaserData {
  uint16_t length = 0;
  float* ranges = nullptr;
  float* intensities = nullptr;

  LaserData() : ranges(new float[kDefaultLaserDataLength]), intensities(new float[kDefaultLaserDataLength]) {}
  LaserData(const uint16_t& l) : ranges(new float[l]), intensities(new float[l]) {}
  LaserData(const LaserData& other) {
    length = other.length;
    ranges = new float[other.length];
    intensities = new float[other.length];
    std::copy(other.ranges, other.ranges + other.length, ranges);
    std::copy(other.intensities, other.intensities + other.length, intensities);
  }
  ~LaserData() {
    delete[] ranges;
    delete[] intensities;
  }

  LaserData& operator=(const LaserData& other) {
    if (this == &other) {
      return *this;
    }

    length = other.length;

    if (ranges) {
      delete[] ranges;
      ranges = nullptr;
    }
    ranges = new float[other.length];
    std::copy(other.ranges, other.ranges + other.length, ranges);

    if (intensities) {
      delete[] intensities;
      intensities = nullptr;
    }
    intensities = new float[other.length];
    std::copy(other.intensities, other.intensities + other.length, intensities);

    return *this;
  }
};

class FlatScan {
 public:
  FlatScan(const ros::NodeHandle& nh);
  ~FlatScan();

  const LaserData& laser_data() { return laser_data_; }

 private:
  bool Initialize();
  bool HandleConfiguration(SetParameter::Request& req, SetParameter::Response& res);
  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void SendCommand(const uint16_t& command, const uint8_t* data);

 private:
  ros::NodeHandle nh_;
  ros::Publisher laser_scan_publisher_;
  ros::ServiceServer configuration_server_;

  // sensor_msgs::LaserData laser_data_;

  SerialPort<FlatScan> com_;
  Protocol protocol_;
  LaserData laser_data_;
};

}  // namespace bea_sensors
