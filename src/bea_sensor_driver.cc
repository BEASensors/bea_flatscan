#include "bea_sensor_driver.h"

#include <ros/console.h>

namespace bea_sensors {

Driver::Driver(const std::string& port, const int& baudrate) {
  com_.registerCallback(this, &Driver::HandleReceivedData);
  com_.connect(port, baudrate);
}

Driver::~Driver() { com_.close(); }

void Driver::HandleReceivedData(char* data, int length) {
  for (int i = 0; i < length; ++i) {
    if (protocol_.InsertByte(data[i]) > 0) {
      DataFrame frame;
      protocol_.GetReceivedFrame(frame);
      ParseDataFrame(frame);
    }
  }
}

void Driver::ParseDataFrame(DataFrame& frame) {
  const uint16_t command = frame.command();
  const uint8_t* data = frame.data();
  switch (command) {
    case CommandFromSensor::MDI:
      break;
    default:
      break;
  }
  delete data;
}

void Driver::SendCommand(const uint16_t& command, const uint8_t* data) {
  int data_length{0};
  switch (command) {
    case CommandToSensor::SET_PARAMETERS:
      data_length = 22;
      break;
    default:
      delete data;
      return;
  }

  uint8_t data_out[data_length + 15];
  int length = protocol_.GenerateFrame(command, data, 1, data_out);
  com_.write((char*)data_out, length);
  sleep(1);
}

}  // namespace bea_sensors
