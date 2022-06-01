#pragma once

#include "protocol.h"
#include "serial_port.h"
#include "serial_port_impl.h"

namespace bea_sensors {

class Driver {
 public:
  Driver(const std::string& port, const int& baudrate = 921600);
  ~Driver();

 private:
  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void SendCommand(const uint16_t& command, const uint8_t* data);

 private:
  SerialPort<Driver> com_;
  Protocol protocol_;
};

}  // namespace bea_sensors
