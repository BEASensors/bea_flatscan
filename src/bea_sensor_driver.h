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
  ~Laser() { delete[] ranges; }
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
