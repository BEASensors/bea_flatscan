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
  const uint16_t length = frame.length();
  switch (command) {
    case CommandFromSensor::MDI: {
      laser_.angle_min = 0.;
      laser_.angle_max = 108. / 180. * M_PI;
      laser_.angle_increment = laser_.angle_max / 400;
      int index{0};
      for (int i = 9; i < length - 1; i += 2) {
        const uint16_t range_in_mm{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
        laser_.ranges[index++] = static_cast<float>(range_in_mm) * 0.001;
        // ROS_INFO("i = %d, range = %d", i, static_cast<int>(range_in_mm));
        // ROS_INFO("range = %f", laser_.ranges[index - 1]);
      }
      ROS_INFO("Received MDI message with %d ranges", index);
      laser_.length = index;
    } break;
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
