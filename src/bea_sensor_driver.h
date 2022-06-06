#pragma once

#include "protocol.h"
#include "serial_port.h"
#include "serial_port_impl.h"

namespace bea_sensors {

struct Laser {
  int length = 0;
  float angle_min = 0.;
  float angle_max = 108. / 180. * M_PI;
  float angle_increment = 0.18 / 180. * M_PI;
  float* ranges;

  Laser() : ranges(new float[400]) {}
  Laser(const int& l) : ranges(new float[l]) {}
  Laser(const Laser& other) {
    length = other.length;
    angle_min = other.angle_min;
    angle_max = other.angle_max;
    angle_increment = other.angle_increment;
    ranges = new float[other.length];
    std::copy(other.ranges, other.ranges + other.length, ranges);
  }
  ~Laser() { delete[] ranges; }

  Laser& operator=(const Laser& other) {
    if (this == &other) {
      return *this;
    }

    if (ranges) {
      delete[] ranges;
      ranges = nullptr;
    }

    ranges = new float[other.length];
    length = other.length;
    std::copy(other.ranges, other.ranges + other.length, ranges);

    return *this;
  }
};

class Driver {
 public:
  Driver(const std::string& port, const int& baudrate = 921600);
  ~Driver();

  const Laser& laser() { return laser_; }

 private:
  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void SendCommand(const uint16_t& command, const uint8_t* data);

 private:
  SerialPort<Driver> com_;
  Protocol protocol_;
  Laser laser_;
};

}  // namespace bea_sensors
