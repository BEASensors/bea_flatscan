#include "flat_scan.h"

#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>

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

  if (detection_field_mode == "HD") {
    ROS_INFO("FlatScan works in HD mode, loading HD configurations");
    resolution_ = 0.18 + 0.09;    // in degrees
    refresh_period_ = 43 * 1e-3;  // in seconds
    number_of_spots_ = 400;
  } else if (detection_field_mode == "HS") {
    ROS_INFO("FlatScan works in HS mode, loading HS configurations");
    resolution_ = 0.74 + 0.09;       // in degrees
    refresh_period_ = 10.75 * 1e-3;  // in seconds
    number_of_spots_ = 100;
  } else {
    ROS_WARN("FlatScan works in %s(UNKNOWN) mode, loading HD configurations by default", detection_field_mode.c_str());
    resolution_ = 0.18 + 0.09;    // in degrees
    refresh_period_ = 43 * 1e-3;  // in seconds
    number_of_spots_ = 400;
  }
  nh_.param("first_angle", first_angle_, static_cast<float>(0.));
  nh_.param("last_angle", last_angle_, static_cast<float>(108.));
  nh_.param("min_range", min_range_, static_cast<float>(0.));
  nh_.param("max_range", max_range_, static_cast<float>(8.));
  nh_.param("frame_id", frame_id_, std::string("laser_link"));

  std::string topic_name;
  nh_.param("topic_name", topic_name, std::string("/scan"));

  laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(topic_name, 1, this);
  configuration_server_ = nh_.advertiseService("configure", &FlatScan::HandleConfiguration, this);

  com_.RegisterCallback(this, &FlatScan::HandleReceivedData);
  com_.Connect(port, baudrate);

  return true;
}

bool FlatScan::HandleConfiguration(Configure::Request& req, Configure::Response& res) {
  if (req.name == "set_baudrate") {
    res.success = true;
    res.description = req.name + " to " + req.value;
    switch (std::stoi(req.value)) {
      case 57600: {
      } break;
      case 115200: {
      } break;
      case 230400: {
      } break;
      case 460800: {
      } break;
      case 921600: {
      } break;
      default: {
        res.success = false;
        res.description = "Unsupported baudrate configuration";
      }
    }
  } else if (req.name == "get_measurement") {
    res.success = true;
    res.description = "switch to " + req.value + " mode";
    if (req.value != "single shot" && req.value != "continuous") {
      res.success = false;
      res.description = "unknown mode: " + req.value;
      return true;
    }
    uint8_t* data{new uint8_t[1]};
    data[0] = req.value == "single shot" ? 0 : 1;
    SendMessage(GET_MEASUREMENTS, 1, data);
    delete[] data;
    data = nullptr;
  } else if (req.name == "get_identity") {
    SendMessage(GET_IDENTITY, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "get_emergency") {
    SendMessage(GET_EMERGENCY, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "get_parameters") {
    SendMessage(GET_PARAMETERS, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "set_parameters") {
  } else if (req.name == "store_parameters") {
    SendMessage(STORE_PARAMETERS, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "reset_mdi_counter") {
    SendMessage(RESET_MDI_COUNTER, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "reset_heartbeat_counter") {
    SendMessage(RESET_HEARTBEAT_COUNTER, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "reset_emergency_counter") {
    SendMessage(RESET_EMERGENCY_COUNTER, 0, nullptr);
    res.success = true;
    res.description = req.name;
  } else if (req.name == "set_led") {
    res.success = true;
    res.description = req.subname + " " + req.value;
    if (req.subname != "set" && req.subname != "blink") {
      res.success = false;
      res.description = req.name + " " + req.subname;
      return true;
    }
    uint8_t* data{new uint8_t[4]};
    const size_t pos{req.value.find(" ")};
    if (req.subname == "set" && pos == req.value.npos) {
      const std::string color1{req.value};
      data[0] = 1;
      if (kColorToNumberMap.find(color1) != kColorToNumberMap.end()) {
        data[1] = kColorToNumberMap.at(color1);
        data[2] = 0;
        data[3] = 0;
      } else {
        res.success = false;
      }
    } else if (req.subname == "blink" && pos != req.value.npos) {
      data[0] = 2;
      const std::string color1{req.value.substr(0, pos)};
      if (kColorToNumberMap.find(color1) != kColorToNumberMap.end()) {
        data[1] = kColorToNumberMap.at(color1);
      } else {
        res.success = false;
      }

      if (res.success) {
        const size_t pos2{req.value.substr(pos + 1, req.value.size()).find(" ")};
        const std::string color2{pos2 == req.value.npos ? req.value.substr(pos + 1, req.value.size()) : req.value.substr(pos + 1, pos2 - pos)};
        const uint8_t frequency{static_cast<uint8_t>(pos2 == req.value.npos ? 1 : std::stoi(req.value.substr(pos2 + 1, req.value.size())))};
        data[3] = frequency;
        if (kColorToNumberMap.find(color2) != kColorToNumberMap.end()) {
          data[2] = kColorToNumberMap.at(color2);
        } else {
          res.success = false;
        }
      }
    } else {
      res.success = false;
      res.description = "Wrong message: " + req.name + " " + req.subname + " " + req.value;
    }

    if (res.success) {
      SendMessage(SET_LED, 4, data);
    }
    delete[] data;
    data = nullptr;
  } else {
    res.success = false;
    res.description = req.name;
  }
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
      sensor_msgs::LaserScan message;
      message.header.stamp = ros::Time::now();
      message.header.frame_id = frame_id_;
      message.angle_min = angles::from_degrees(first_angle_);
      message.angle_max = angles::from_degrees(last_angle_);
      message.angle_increment = angles::from_degrees(resolution_);
      message.range_min = min_range_;
      message.range_max = max_range_;
      message.scan_time = refresh_period_;
      message.time_increment = refresh_period_ / number_of_spots_;
      if (length == 809) {
        for (int i = 9; i < length - 1; i += 2) {
          const uint16_t range_in_mm{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
          message.ranges.push_back(static_cast<float>(range_in_mm) * 0.001);
        }
      } else if (length == 1609) {
        for (int i = 9; i < 809; i += 2) {
          const uint16_t range_in_mm{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
          message.ranges.push_back(static_cast<float>(range_in_mm) * 0.001);
        }
        for (int i = 809; i < length - 1; i += 2) {
          const uint16_t intensity{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
          message.intensities.push_back(static_cast<float>(intensity));
        }
      }
      laser_scan_publisher_.publish(message);
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

void FlatScan::SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data) {
  // int data_length{0};
  // switch (command) {
  //   case CommandToSensor::SET_BAUDRATE: {
  //     data_length = 1;
  //   } break;
  //   case CommandToSensor::GET_MEASUREMENTS: {
  //     data_length = 1;
  //   } break;
  //   case CommandToSensor::GET_IDENTITY: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::GET_EMERGENCY: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::GET_PARAMETERS: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::SET_PARAMETERS: {
  //     data_length = 22;
  //   } break;
  //   case CommandToSensor::STORE_PARAMETERS: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::RESET_MDI_COUNTER: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::RESET_HEARTBEAT_COUNTER: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::RESET_EMERGENCY_COUNTER: {
  //     data_length = 0;
  //   } break;
  //   case CommandToSensor::SET_LED: {
  //     data_length = 4;
  //   } break;
  //   default:
  //     ROS_WARN("Unknown CMD to sensor: %d", command);
  //     return;
  // }

  uint8_t data_out[data_length + kFrameMinimalLength];
  uint16_t length = protocol_.GenerateFrame(command, data, data_length, data_out);
  if (length < kFrameMinimalLength) {
    return;
  }
  com_.Write((char*)data_out, length);
  usleep(5000);
}

}  // namespace bea_sensors
