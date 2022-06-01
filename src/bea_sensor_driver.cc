#include "bea_sensor_driver.h"

#include <ros/console.h>

namespace bea_sensors {

Driver::Driver(const std::string& port, const int& baudrate) {
  com_.RegisterCallback(this, &Driver::HandleReceivedData);
  com_.Connect(port, baudrate);
}

Driver::~Driver() { com_.Close(); }

void Driver::HandleReceivedData(char* data, int length) {
  for (int i = 0; i < length; ++i) {
    if (protocol_.InsertByte(data[i]) > 0) {
      DataFrame frame;
      if (!protocol_.GetReceivedFrame(frame)) {
        ROS_ERROR("Get received frame error");
        continue;
      }
      ParseDataFrame(frame);
    }
  }
}

void Driver::ParseDataFrame(DataFrame& frame) {
  const uint16_t command = frame.command();
  const uint8_t* data = frame.data();
  switch (command) {
    case CommandFromSensor::MDI:
      ROS_INFO("Received MDI message");
      break;
    default:
      break;
  }
}

void Driver::SendCommand(const uint16_t& command, const uint8_t* data) {
  int data_length{0};
  switch (command) {
    case CommandToSensor::SET_PARAMETERS:
      data_length = 22;
      break;
    default:
      return;
  }

  uint8_t data_out[data_length + 15];
  int length = protocol_.GenerateFrame(command, data, 1, data_out);
  com_.Write((char*)data_out, length);
  sleep(1);
}

}  // namespace bea_sensors
