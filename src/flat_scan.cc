#include "flat_scan.h"

#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>

#include "bea_sensors/Emergency.h"
#include "bea_sensors/Heartbeat.h"

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
    parameters_.mode = 1;
    resolution_ = kHighDensityResolution;         // in degrees
    refresh_period_ = kHighDensityRefreshPeriod;  // in seconds
    parameters_.number_of_spots = 400;
  } else if (detection_field_mode == "HS") {
    ROS_INFO("FlatScan works in HS mode, loading HS configurations");
    parameters_.mode = 0;
    resolution_ = kHighSpeedResolution;         // in degrees
    refresh_period_ = kHighSpeedRefreshPeriod;  // in seconds
    parameters_.number_of_spots = 100;
  } else {
    ROS_WARN("FlatScan works in %s(UNKNOWN) mode, loading HD configurations by default", detection_field_mode.c_str());
    parameters_.mode = 1;
    resolution_ = kHighDensityResolution;         // in degrees
    refresh_period_ = kHighDensityRefreshPeriod;  // in seconds
    parameters_.number_of_spots = 400;
  }
  nh_.param("first_angle", first_angle_, static_cast<float>(0.));
  nh_.param("last_angle", last_angle_, static_cast<float>(108.));
  nh_.param("min_range", min_range_, static_cast<float>(0.));
  nh_.param("max_range", max_range_, static_cast<float>(8.));
  nh_.param("frame_id", frame_id_, std::string("laser_link"));

  std::string scan_topic, heartbeat_topic, emergency_topic;
  nh_.param("scan_topic", scan_topic, std::string("/scan"));
  nh_.param("heartbeat_topic", heartbeat_topic, std::string("/heartbeat"));
  nh_.param("emergency_topic", emergency_topic, std::string("/emergency"));

  laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 1, this);
  heartbeat_publisher_ = nh_.advertise<Heartbeat>(heartbeat_topic, 1, this);
  emergency_publisher_ = nh_.advertise<Emergency>(emergency_topic, 1, this);
  configuration_server_ = nh_.advertiseService("configure", &FlatScan::HandleConfiguration, this);

  com_.RegisterCallback(this, &FlatScan::HandleReceivedData);
  com_.Connect(port, baudrate);

  return true;
}

bool FlatScan::HandleConfiguration(Configure::Request& req, Configure::Response& res) {
  if (req.command == "set_baudrate") {
    HandleSetBaudrate(req, res);
  } else if (req.command == "get_measurements") {
    HandleGetMeasurements(req, res);
  } else if (req.command == "get_identity") {
    SendMessage(GET_IDENTITY, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "get_emergency") {
    SendMessage(GET_EMERGENCY, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "get_parameters") {
    SendMessage(GET_PARAMETERS, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "set_parameters") {
    HandleSetParameters(req, res);
  } else if (req.command == "store_parameters") {
    SendMessage(STORE_PARAMETERS, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "reset_mdi_counter") {
    SendMessage(RESET_MDI_COUNTER, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "reset_heartbeat_counter") {
    SendMessage(RESET_HEARTBEAT_COUNTER, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "reset_emergency_counter") {
    SendMessage(RESET_EMERGENCY_COUNTER, 0, nullptr);
    res.success = true;
    res.description = req.command;
  } else if (req.command == "set_led") {
    HandleSetLed(req, res);
  } else {
    res.success = false;
    res.description = req.command;
  }
  return true;
}

void FlatScan::HandleSetBaudrate(Configure::Request& req, Configure::Response& res) {
  res.success = true;
  res.description = req.command + " to " + req.value;
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

void FlatScan::HandleSetParameters(Configure::Request& req, Configure::Response& res) {
  res.success = true;
  res.description = "set " + req.subcommand + " to " + req.value;
  if (kParameterMap.find(req.subcommand) == kParameterMap.end()) {
    res.success = false;
    res.description = "no " + req.subcommand;
    return;
  }

  uint8_t* data{new uint8_t[22]{0}};
  data[1] = parameters_.temperature;
  data[2] = parameters_.information;
  data[3] = parameters_.mode;
  data[4] = parameters_.optimization;
  data[8] = static_cast<uint8_t>(parameters_.number_of_spots & 0xff);
  data[9] = static_cast<uint8_t>((parameters_.number_of_spots & 0xff00) >> 8);
  data[14] = static_cast<uint8_t>(parameters_.angle_first & 0xff);
  data[15] = static_cast<uint8_t>((parameters_.angle_first & 0xff00) >> 8);
  data[16] = static_cast<uint8_t>(parameters_.angle_last & 0xff);
  data[17] = static_cast<uint8_t>((parameters_.angle_last & 0xff00) >> 8);
  data[18] = parameters_.counter;
  data[19] = parameters_.heartbeat_period;
  data[20] = parameters_.facet;
  data[21] = parameters_.averaging;

  if (req.subcommand != "spots" && req.subcommand != "angle_first" && req.subcommand != "angle_last") {
    data[kParameterMap.at(req.subcommand)] = static_cast<uint8_t>(std::stoi(req.value));
  } else {
    const uint16_t value{static_cast<uint16_t>(std::stoi(req.value))};
    data[kParameterMap.at(req.subcommand)] = static_cast<uint8_t>(value & 0xff);
    data[kParameterMap.at(req.subcommand) + 1] = static_cast<uint8_t>((value & 0xff00) >> 8);
  }

  SendMessage(SET_PARAMETERS, 22, data);
  delete[] data;
  data = nullptr;
}

void FlatScan::HandleSetLed(Configure::Request& req, Configure::Response& res) {
  res.success = true;
  res.description = req.subcommand + " " + req.value;
  if (req.subcommand != "set" && req.subcommand != "blink") {
    res.success = false;
    res.description = req.command + " " + req.subcommand;
    return;
  }

  const size_t pos{req.value.find(" ")};
  if (req.subcommand == "set" && pos == req.value.npos) {
    uint8_t* data{new uint8_t[2]};
    const std::string color1{req.value};
    data[0] = 1;
    if (kColorMap.find(color1) != kColorMap.end()) {
      data[1] = kColorMap.at(color1);
      SendMessage(SET_LED, 2, data);
    } else {
      res.success = false;
    }
    delete[] data;
    data = nullptr;
  } else if (req.subcommand == "blink" && pos != req.value.npos) {
    uint8_t* data{new uint8_t[4]};
    data[0] = 2;
    const std::string color1{req.value.substr(0, pos)};
    if (kColorMap.find(color1) != kColorMap.end()) {
      data[1] = kColorMap.at(color1);
    } else {
      res.success = false;
    }

    if (res.success) {
      const std::string substr{req.value.substr(pos + 1, req.value.size())};
      const size_t pos2{substr.find(" ")};
      const std::string color2{pos2 == substr.npos ? substr : substr.substr(0, pos2)};
      const std::string frequency_str{substr.substr(pos2 + 1, substr.size() - pos2)};
      const uint8_t frequency{static_cast<uint8_t>(pos2 == substr.npos ? 1 : std::stoi(frequency_str))};
      data[3] = frequency;
      if (kColorMap.find(color2) != kColorMap.end()) {
        data[2] = kColorMap.at(color2);
        SendMessage(SET_LED, 4, data);
      } else {
        res.success = false;
      }
    }
    delete[] data;
    data = nullptr;
  } else {
    res.success = false;
    res.description = "Wrong message: " + req.command + " " + req.subcommand + " " + req.value;
  }
}

void FlatScan::SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data) {
  std::unique_lock<std::mutex> lock(mutex_);
  message_sent_ = false;
  lock.unlock();
  uint8_t data_out[data_length + kFrameMinimalLength];
  uint16_t length = protocol_.GenerateFrame(command, data, data_length, data_out);
  if (length < kFrameMinimalLength) {
    return;
  }
  while (!message_sent_) {
    ROS_INFO("send message");
    com_.Write((char*)data_out, length);
    sleep(2);
  }
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
      ParseMdiMessage(data, length);
    } break;
    case CommandFromSensor::SEND_IDENTITY: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ParseSendIdentityMessage(data, length);
    } break;
    case CommandFromSensor::SEND_PARAMETERS: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ParseSendParametersMessage(data, length);
    } break;
    case CommandFromSensor::HEARTBEAT: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ParseHeartbeatMessage(data, length);
    } break;
    case CommandFromSensor::EMERGENCY: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ParseEmergencyMessage(data, length);
    } break;
    case CommandToSensor::SET_BAUDRATE: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ROS_INFO("Set baudrate succeeded: %i", data[0]);
    } break;
    case CommandToSensor::STORE_PARAMETERS: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ROS_INFO("Store parameters succeeded");
    } break;
    case CommandToSensor::RESET_MDI_COUNTER: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ROS_INFO("Reset MDI counter succeeded");
    } break;
    case CommandToSensor::RESET_HEARTBEAT_COUNTER: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ROS_INFO("Reset heartbeat counter succeeded");
    } break;
    case CommandToSensor::RESET_EMERGENCY_COUNTER: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ROS_INFO("Reset emergency counter succeeded");
    } break;
    case CommandToSensor::SET_LED: {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
      ROS_INFO("Set LED succeeded");
    } break;
    default:
      ROS_WARN("Unknown CMD from sensor: %i", command);
      return;
  }
}

void FlatScan::ParseMdiMessage(const uint8_t* data, const int& length) {
  uint16_t start_index{0};
  start_index += (parameters_.counter > 0 ? 6 : 0);
  start_index += (parameters_.temperature > 0 ? 2 : 0);
  start_index += (parameters_.facet > 0 ? 1 : 0);
  const uint16_t distance_length{static_cast<uint16_t>(parameters_.number_of_spots * 2)};

  uint16_t total_length{static_cast<uint16_t>(start_index + distance_length)};
  total_length += parameters_.information == 2 ? distance_length : 0;
  if (total_length != length) {
    ROS_ERROR("MDI data length mismatch (should be %i but get %i)", total_length, length);
    return;
  }

  sensor_msgs::LaserScan message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = frame_id_;
  message.angle_min = angles::from_degrees(first_angle_);
  message.angle_max = angles::from_degrees(last_angle_);
  message.angle_increment = angles::from_degrees(resolution_);
  message.range_min = min_range_;
  message.range_max = max_range_;
  message.scan_time = refresh_period_;
  message.time_increment = refresh_period_ / parameters_.number_of_spots;
  switch (parameters_.information) {
    case 0: {
      for (int i = start_index; i < length - 1; i += 2) {
        const uint16_t range_in_mm{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
        message.ranges.push_back(static_cast<float>(range_in_mm) * 0.001);
      }
    } break;
    case 1: {
      for (int i = start_index; i < length - 1; i += 2) {
        const uint16_t intensity{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
        message.intensities.push_back(static_cast<float>(intensity));
      }
    } break;
    case 2: {
      for (int i = start_index; i < start_index + distance_length - 1; i += 2) {
        const uint16_t range_in_mm{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
        message.ranges.push_back(static_cast<float>(range_in_mm) * 0.001);
      }
      for (int i = start_index + distance_length; i < length - 1; i += 2) {
        const uint16_t intensity{static_cast<uint16_t>(data[i] | (data[i + 1] << 8))};
        message.intensities.push_back(static_cast<float>(intensity));
      }
    } break;
    default:
      return;
  }
  laser_scan_publisher_.publish(message);
}

void FlatScan::ParseSendIdentityMessage(const uint8_t* data, const int& length) {
  const uint32_t product_part_number{static_cast<uint32_t>(data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24)};
  const uint8_t software_version{data[4]};
  const uint8_t software_revision{data[5]};
  const uint8_t software_prototype{data[6]};
  const uint32_t serial_number{static_cast<uint32_t>(data[7] | data[8] << 8 | data[9] << 16 | data[10] << 24)};
  ROS_INFO("product_part_number: %i", product_part_number);
  ROS_INFO("software_version: %i", software_version);
  ROS_INFO("software_revision: %i", software_revision);
  ROS_INFO("software_prototype: %i", software_prototype);
  ROS_INFO("serial_number: %i", serial_number);
}

void FlatScan::ParseSendParametersMessage(const uint8_t* data, const int& length) {
  const uint32_t verification_code{static_cast<uint32_t>(data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24)};
  const uint16_t charge{static_cast<uint16_t>(data[4] | data[5] << 8)};

  parameters_.temperature = data[7];
  parameters_.information = data[8];
  parameters_.mode = data[9];
  parameters_.optimization = data[10];
  parameters_.number_of_spots = static_cast<uint16_t>(data[14] | data[15] << 8);
  parameters_.angle_first = data[20] | data[21] << 8;
  parameters_.angle_last = data[22] | data[23] << 8;
  parameters_.counter = data[24];
  parameters_.heartbeat_period = data[25];
  parameters_.facet = data[26];
  parameters_.averaging = data[27];

  if (parameters_.mode == 0) {
    resolution_ = kHighSpeedResolution;
    refresh_period_ = kHighSpeedRefreshPeriod;
  } else if (parameters_.mode == 1) {
    resolution_ = kHighDensityResolution;
    refresh_period_ = kHighDensityRefreshPeriod;
  }
  first_angle_ = static_cast<float>(parameters_.angle_first) * 1e-2;
  last_angle_ = static_cast<float>(parameters_.angle_last) * 1e-2;

  ROS_INFO("verification_code: %i", verification_code);
  ROS_INFO("charge: %i", charge);
  ROS_INFO("temperature_enabled: %i", parameters_.temperature);
  ROS_INFO("information: %i", parameters_.information);
  ROS_INFO("detection_field_mode: %i", parameters_.mode);
  ROS_INFO("optimization: %i", parameters_.optimization);
  ROS_INFO("number_of_spots: %i", parameters_.number_of_spots);
  ROS_INFO("angle_first: %f", first_angle_);
  ROS_INFO("angle_last: %f", last_angle_);
  ROS_INFO("counter_enabled: %i", parameters_.counter);
  ROS_INFO("heartbeat_period: %i", parameters_.heartbeat_period);
  ROS_INFO("facet_enabled: %i", parameters_.facet);
  ROS_INFO("averaging_setting: %i", parameters_.averaging);
}

void FlatScan::ParseHeartbeatMessage(const uint8_t* data, const int& length) {
  Heartbeat message;
  message.count = static_cast<uint16_t>(data[4] | data[5] << 8);
  if (heartbeat_publisher_.getNumSubscribers() > 0) {
    heartbeat_publisher_.publish(message);
  }
  // ROS_INFO("Heartbeat count: %i", message.count));
}

void FlatScan::ParseEmergencyMessage(const uint8_t* data, const int& length) {
  Emergency message;
  message.count = static_cast<uint16_t>(data[4] | data[5] << 8);
  message.rs485_error = static_cast<uint16_t>(data[6] | data[7] << 8);
  message.sensor_error = static_cast<uint16_t>(data[8] | data[9] << 8);
  if (emergency_publisher_.getNumSubscribers() > 0) {
    emergency_publisher_.publish(message);
  }
  ROS_INFO("Emergency count: %i, error code: %i %i", message.count, message.rs485_error, message.sensor_error);
}

}  // namespace bea_sensors
