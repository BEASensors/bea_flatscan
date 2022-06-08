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
    HandleSetBaudrate(req, res);
  } else if (req.name == "get_measurements") {
    HandleGetMeasurements(req, res);
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
    HandleSetParameters(req, res);
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
    HandleSetLed(req, res);
  } else {
    res.success = false;
    res.description = req.name;
  }
  return true;
}

void FlatScan::HandleSetBaudrate(Configure::Request& req, Configure::Response& res) {
  res.success = true;
  res.description = req.name + " to " + req.value;
  uint8_t* data{new uint8_t[1]};
  if (kBaudrateMap.find(req.value) != kBaudrateMap.end()) {
    data[0] = kBaudrateMap.at(req.value);
  } else {
    res.success = false;
    res.description = "Unsupported baudrate configuration";
  }
  if (res.success) {
    SendMessage(SET_BAUDRATE, 1, data);
  }
  delete[] data;
  data = nullptr;
}

void FlatScan::HandleGetMeasurements(Configure::Request& req, Configure::Response& res) {
  res.success = true;
  res.description = "switch to " + req.value + " mode";
  if (req.value != "single shot" && req.value != "continuous") {
    res.success = false;
    res.description = "unknown mode: " + req.value;
    return;
  }
  uint8_t* data{new uint8_t[1]};
  data[0] = req.value == "single shot" ? 0 : 1;
  SendMessage(GET_MEASUREMENTS, 1, data);
  delete[] data;
  data = nullptr;
}

void FlatScan::HandleSetParameters(Configure::Request& req, Configure::Response& res) {}

void FlatScan::HandleSetLed(Configure::Request& req, Configure::Response& res) {
  res.success = true;
  res.description = req.subname + " " + req.value;
  if (req.subname != "set" && req.subname != "blink") {
    res.success = false;
    res.description = req.name + " " + req.subname;
    return;
  }
  uint8_t* data{new uint8_t[4]};
  const size_t pos{req.value.find(" ")};
  if (req.subname == "set" && pos == req.value.npos) {
    const std::string color1{req.value};
    data[0] = 1;
    if (kColorMap.find(color1) != kColorMap.end()) {
      data[1] = kColorMap.at(color1);
      data[2] = 0;
      data[3] = 0;
    } else {
      res.success = false;
    }
  } else if (req.subname == "blink" && pos != req.value.npos) {
    data[0] = 2;
    const std::string color1{req.value.substr(0, pos)};
    if (kColorMap.find(color1) != kColorMap.end()) {
      data[1] = kColorMap.at(color1);
    } else {
      res.success = false;
    }

    if (res.success) {
      const size_t pos2{req.value.substr(pos + 1, req.value.size()).find(" ")};
      const std::string color2{pos2 == req.value.npos ? req.value.substr(pos + 1, req.value.size()) : req.value.substr(pos + 1, pos2 - pos)};
      const uint8_t frequency{static_cast<uint8_t>(pos2 == req.value.npos ? 1 : std::stoi(req.value.substr(pos2 + 1, req.value.size())))};
      data[3] = frequency;
      if (kColorMap.find(color2) != kColorMap.end()) {
        data[2] = kColorMap.at(color2);
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
    case CommandToSensor::SET_BAUDRATE: {
      ROS_INFO("Set baudrate succeeded: %d", data[0]);
    } break;
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
      const uint32_t product_part_number{static_cast<uint32_t>(data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24)};
      const uint8_t software_version{data[4]};
      const uint8_t software_revision{data[5]};
      const uint8_t software_prototype{data[6]};
      const uint32_t serial_number{static_cast<uint32_t>(data[7] | data[8] << 8 | data[9] << 16 | data[10] << 24)};
      // ROS_INFO("product_part_number: %d", static_cast<int>(product_part_number));
      // ROS_INFO("software_version: %d", static_cast<int>(software_version));
      // ROS_INFO("software_revision: %d", static_cast<int>(software_revision));
      // ROS_INFO("software_prototype: %d", static_cast<int>(software_prototype));
      // ROS_INFO("serial_number: %d", static_cast<int>(serial_number));
    } break;
    case CommandFromSensor::SEND_PARAMETERS: {
      const uint32_t verification_code{static_cast<uint32_t>(data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24)};
      const uint16_t charge{static_cast<uint16_t>(data[4] | data[5] << 8)};
      const uint8_t temperature_enabled{data[7]};
      const uint8_t information{data[8]};
      const uint8_t detection_field_mode{data[9]};
      const uint8_t optimization{data[10]};
      const uint16_t number_of_spots{static_cast<uint16_t>(data[14] | data[15] << 8)};
      const uint16_t angle_first{static_cast<uint16_t>(data[20] | data[21] << 8)};
      const uint16_t angle_last{static_cast<uint16_t>(data[22] | data[23] << 8)};
      const uint8_t counter_enabled{data[24]};
      const uint8_t heartbeat_period{data[25]};
      const uint8_t facet_enabled{data[26]};
      const uint8_t averaging_setting{data[27]};
      // ROS_INFO("verification_code: %d", static_cast<int>(verification_code));
      // ROS_INFO("charge: %d", static_cast<int>(charge));
      // ROS_INFO("temperature_enabled: %d", static_cast<int>(temperature_enabled));
      // ROS_INFO("information: %d", static_cast<int>(information));
      // ROS_INFO("detection_field_mode: %d", static_cast<int>(detection_field_mode));
      // ROS_INFO("optimization: %d", static_cast<int>(optimization));
      // ROS_INFO("number_of_spots: %d", static_cast<int>(number_of_spots));
      // ROS_INFO("angle_first: %d", static_cast<int>(angle_first));
      // ROS_INFO("angle_last: %d", static_cast<int>(angle_last));
      // ROS_INFO("counter_enabled: %d", static_cast<int>(counter_enabled));
      // ROS_INFO("heartbeat_period: %d", static_cast<int>(heartbeat_period));
      // ROS_INFO("facet_enabled: %d", static_cast<int>(facet_enabled));
      // ROS_INFO("averaging_setting: %d", static_cast<int>(averaging_setting));
    } break;
    case CommandFromSensor::HEARTBEAT: {
      const uint16_t heartbeat_count{static_cast<uint16_t>(data[4] | data[5] << 8)};
      // ROS_INFO("Heartbeat count: %d", static_cast<int>(heartbeat_count));
    } break;
    case CommandFromSensor::EMERGENCY: {
      const uint16_t emergency_count{static_cast<uint16_t>(data[4] | data[5] << 8)};
      const uint16_t rs485_error_code{static_cast<uint16_t>(data[6] | data[7] << 8)};
      const uint16_t sensor_error_code{static_cast<uint16_t>(data[8] | data[9] << 8)};
      // ROS_INFO("Emergency count: %d, error code: %d %d", static_cast<int>(emergency_count), static_cast<int>(rs485_error_code),
      //          static_cast<int>(sensor_error_code));
    } break;
    case CommandToSensor::STORE_PARAMETERS: {
      ROS_INFO("Store parameters succeeded");
    } break;
    case CommandToSensor::RESET_MDI_COUNTER: {
      ROS_INFO("Reset MDI counter succeeded");
    } break;
    case CommandToSensor::RESET_HEARTBEAT_COUNTER: {
      ROS_INFO("Reset heartbeat counter succeeded");
    } break;
    case CommandToSensor::RESET_EMERGENCY_COUNTER: {
      ROS_INFO("Reset emergency counter succeeded");
    } break;
    case CommandToSensor::SET_LED: {
      ROS_INFO("Set LED succeeded");
    } break;
    default:
      ROS_WARN("Unknown CMD from sensor: %d", command);
      return;
  }
}

void FlatScan::SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data) {
  uint8_t data_out[data_length + kFrameMinimalLength];
  uint16_t length = protocol_.GenerateFrame(command, data, data_length, data_out);
  if (length < kFrameMinimalLength) {
    return;
  }
  com_.Write((char*)data_out, length);
  usleep(5000);
}

}  // namespace bea_sensors
