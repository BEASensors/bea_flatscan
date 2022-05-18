#pragma once

#include "protocol.h"
#include "rtserialport.h"
#include "rtserialport_impl.h"

namespace bea_sensors {

class Driver {
 public:
  Driver(const std::string& port, const int& baudrate = 57600);
  ~Driver();

 private:
  void HandleReceivedData(char* data, int length);
  void ParseDataFrame(DataFrame& frame);
  void SendCommand(const uint16_t& command, const uint8_t* data);

 private:
  RtSerialPort<Driver> com_;
  Protocol protocol_;
};

}  // namespace bea_sensors
