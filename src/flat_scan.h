#pragma once

#include "protocol.h"
#include "serial_port.h"
#include "serial_port_impl.h"

namespace bea_sensors {

constexpr int kDefaultLaserDataLength = 400;

struct LaserData {
  int length = 0;
  float* ranges = nullptr;
  float* intensities = nullptr;

  LaserData() : ranges(new float[kDefaultLaserDataLength]), intensities(new float[kDefaultLaserDataLength]) {}
  LaserData(const int& l) : ranges(new float[l]), intensities(new float[l]) {}
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
  FlatScan(const std::string& port, const int& baudrate = 921600);
  ~FlatScan();

  const LaserData& laser_data() { return laser_data_; }

 private:
  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void SendCommand(const uint16_t& command, const uint8_t* data);

 private:
  SerialPort<FlatScan> com_;
  Protocol protocol_;
  LaserData laser_data_;
};

}  // namespace bea_sensors
