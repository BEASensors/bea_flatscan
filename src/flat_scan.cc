#include "flat_scan.h"

#include <angles/angles.h>

namespace bea_sensors {

FlatScan::FlatScan(const ros::NodeHandle& nh_) : nh_(nh_) { Initialize(); }

FlatScan::~FlatScan() { com_.Close(); }

bool FlatScan::Initialize() {
  std::string port;
  nh_.param("port", port, std::string("/dev/ttyUSB0"));

  int baudrate;
  nh_.param("baudrate", baudrate, 921600);

  std::string detection_field_mode;
  nh_.param("detection_field_mode", detection_field_mode, std::string("HD"));

  int number_of_spots;
  float resolution, refresh_period;
  if (detection_field_mode == "HD") {
    ROS_INFO("FlatScan works in HD mode, loading HD configurations");
    resolution = 0.18 + 0.09;    // in degrees
    refresh_period = 43 * 1e-3;  // in seconds
    number_of_spots = 400;
  } else if (detection_field_mode == "HS") {
    ROS_INFO("FlatScan works in HS mode, loading HS configurations");
    resolution = 0.74 + 0.09;       // in degrees
    refresh_period = 10.75 * 1e-3;  // in seconds
    number_of_spots = 100;
  } else {
    ROS_WARN("FlatScan works in UNKNOWN mode, loading HD configurations by default");
    resolution = 0.18 + 0.09;    // in degrees
    refresh_period = 43 * 1e-3;  // in seconds
    number_of_spots = 400;
  }

  float first_angle, last_angle;
  nh_.param("first_angle", first_angle, static_cast<float>(0.));
  nh_.param("last_angle", last_angle, static_cast<float>(108.));

  float min_range, max_range;
  nh_.param("min_range", min_range, static_cast<float>(0.));
  nh_.param("max_range", max_range, static_cast<float>(8.));

  std::string frame_id;
  nh_.param("frame_id", frame_id, std::string("laser_link"));

  std::string topic_name;
  nh_.param("topic_name", topic_name, std::string("/scan"));

  laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(topic_name, 1, this);
  configuration_server_ = nh_.advertiseService("set_parameter", &FlatScan::HandleConfiguration, this);

  com_.RegisterCallback(this, &FlatScan::HandleReceivedData);
  com_.Connect(port, baudrate);

  return true;
}

bool FlatScan::HandleConfiguration(SetParameter::Request& req, SetParameter::Response& res) {
  res.success = true;
  res.description = req.name;
  return true;
}

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
      // ROS_INFO("Received MDI message with %d ranges", index);
      laser_data_.length = index;
    } break;
    case CommandFromSensor::SEND_IDENTITY: {
    } break;
    case CommandFromSensor::SEND_PARAMETERS: {
    } break;
    case CommandFromSensor::HEARTBEAT: {
    } break;
    case CommandFromSensor::EMERGENCY: {
    } break;
    default:
      ROS_WARN("Unknown CMD from sensor: %d", command);
      return;
  }
}

void FlatScan::SendCommand(const uint16_t& command, /*const uint16_t& data_length,*/ const uint8_t* data) {
  int data_length{0};
  switch (command) {
    case CommandToSensor::SET_BAUDRATE: {
      data_length = 1;
    } break;
    case CommandToSensor::GET_MEASUREMENTS: {
      data_length = 1;
    } break;
    case CommandToSensor::GET_IDENTITY: {
      data_length = 0;
    } break;
    case CommandToSensor::GET_EMERGENCY: {
      data_length = 0;
    } break;
    case CommandToSensor::GET_PARAMETERS: {
      data_length = 0;
    } break;
    case CommandToSensor::SET_PARAMETERS: {
      data_length = 22;
    } break;
    case CommandToSensor::STORE_PARAMETERS: {
      data_length = 0;
    } break;
    case CommandToSensor::RESET_MDI_COUNTER: {
      data_length = 0;
    } break;
    case CommandToSensor::RESET_HEARTBEAT_COUNTER: {
      data_length = 0;
    } break;
    case CommandToSensor::RESET_EMERGENCY_COUNTER: {
      data_length = 0;
    } break;
    case CommandToSensor::SET_LED: {
      data_length = 4;
    } break;
    default:
      ROS_WARN("Unknown CMD to sensor: %d", command);
      return;
  }

  uint8_t data_out[data_length + kFrameMinimalLength];
  uint16_t length = protocol_.GenerateFrame(command, data, data_length, data_out);
  if (length < kFrameMinimalLength) {
    return;
  }
  com_.Write((char*)data_out, length);
  usleep(5000);
}

}  // namespace bea_sensors
