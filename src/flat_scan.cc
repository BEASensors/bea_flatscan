#include "flat_scan.h"

#include <ros/console.h>

namespace bea_sensors {

FlatScan::FlatScan(const std::string& port, const int& baudrate) {
  com_.RegisterCallback(this, &FlatScan::HandleReceivedData);
  com_.Connect(port, baudrate);
}

FlatScan::~FlatScan() { com_.Close(); }

void FlatScan::HandleReceivedData(char* data, int length) {
  for (int i = 0; i < length; ++i) {
    if (protocol_.InsertByte(data[i]) > 0) {
      DataFrame frame;
      if (!protocol_.GetLatestFrame(frame)) {
        ROS_ERROR("Get received frame error");
        continue;
      }
      ParseDataFrame(frame);
    }
  }
}

void FlatScan::ParseDataFrame(DataFrame& frame) {
  const uint16_t command = frame.command();
  const uint8_t* data = frame.data();
  const uint16_t length = frame.length();
  switch (command) {
    case CommandFromSensor::MDI: {
      int index{0};
      for (int i = 9; i < length - 1; i += 2) {
        const uint16_t range_in_mm{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
        laser_data_.ranges[index++] = static_cast<float>(range_in_mm) * 0.001;
      }
      ROS_INFO("Received MDI message with %d ranges", index);
      laser_data_.length = index;
    } break;
    default:
      break;
  }
}

void FlatScan::SendCommand(const uint16_t& command, const uint8_t* data) {
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
