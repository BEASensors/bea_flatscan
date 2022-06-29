#include "parser.h"

#include <angles/angles.h>
#include <ros/console.h>

namespace bea_sensors {

Parser::Parser() {}

Parser::~Parser() {}

bool Parser::GenerateDataFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                               std::string& description, DataFrame& frame) {
  success = true;
  description = command;
  if (command == "set_baudrate") {
    GenerateSetBaudrateFrame(command, subcommand, value, success, description, frame);
  } else if (command == "get_measurements") {
    GenerateGetMeasurementsFrame(command, subcommand, value, success, description, frame);
  } else if (command == "get_identity") {
    frame = DataFrame(GET_IDENTITY, nullptr, 0);
  } else if (command == "get_emergency") {
    frame = DataFrame(GET_EMERGENCY, nullptr, 0);
  } else if (command == "get_parameters") {
    frame = DataFrame(GET_PARAMETERS, nullptr, 0);
  } else if (command == "set_parameters") {
    GenerateSetParametersFrame(command, subcommand, value, success, description, frame);
  } else if (command == "store_parameters") {
    frame = DataFrame(STORE_PARAMETERS, nullptr, 0);
  } else if (command == "reset_mdi_counter") {
    frame = DataFrame(RESET_MDI_COUNTER, nullptr, 0);
  } else if (command == "reset_heartbeat_counter") {
    frame = DataFrame(RESET_HEARTBEAT_COUNTER, nullptr, 0);
  } else if (command == "reset_emergency_counter") {
    frame = DataFrame(RESET_EMERGENCY_COUNTER, nullptr, 0);
  } else if (command == "set_led") {
    GenerateSetLedFrame(command, subcommand, value, success, description, frame);
  } else {
    success = false;
  }
  return success;
}

void Parser::GenerateSetBaudrateFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                      std::string& description, DataFrame& frame) const {
  success = true;
  description = command + " to " + value;
  if (kBaudrateMap.find(value) == kBaudrateMap.end()) {
    success = false;
    description = "Unsupported baudrate configuration";
    return;
  }

  uint8_t* data{new uint8_t[1]};
  data[0] = kBaudrateMap.at(value);
  frame = DataFrame(SET_BAUDRATE, data, 1);
  delete[] data;
  data = nullptr;
}

void Parser::GenerateGetMeasurementsFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                          std::string& description, DataFrame& frame) const {
  success = true;
  description = "switch to " + value + " mode";
  if (value != "single shot" && value != "continuous") {
    success = false;
    description = "unknown mode: " + value;
    return;
  }
  uint8_t* data{new uint8_t[1]};
  data[0] = value == "single shot" ? 0 : 1;
  frame = DataFrame(GET_MEASUREMENTS, data, 1);
  delete[] data;
  data = nullptr;
}

void Parser::GenerateSetParametersFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                        std::string& description, DataFrame& frame) const {
  success = true;
  description = "set " + subcommand + " to " + value;
  if (subcommand == "") {
    description = "initialize configuration";
  } else if (kParameterMap.find(subcommand) == kParameterMap.end()) {
    success = false;
    description = "no " + subcommand;
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

  if (subcommand != "") {
    if (subcommand != "spots" && subcommand != "angle_first" && subcommand != "angle_last") {
      data[kParameterMap.at(subcommand)] = static_cast<uint8_t>(std::stoi(value));
    } else {
      const uint16_t data_value{static_cast<uint16_t>(std::stoi(value))};
      data[kParameterMap.at(subcommand)] = static_cast<uint8_t>(data_value & 0xff);
      data[kParameterMap.at(subcommand) + 1] = static_cast<uint8_t>((data_value & 0xff00) >> 8);
    }
  }

  frame = DataFrame(SET_PARAMETERS, data, 22);
  delete[] data;
  data = nullptr;
}

void Parser::GenerateSetLedFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                 std::string& description, DataFrame& frame) const {
  success = true;
  description = subcommand + " " + value;
  if (subcommand != "set" && subcommand != "blink") {
    success = false;
    description = command + " " + subcommand;
    return;
  }

  uint8_t* data{new uint8_t[4]{0}};
  const size_t pos{value.find(" ")};
  if (subcommand == "set" && pos == value.npos) {
    const std::string color1{value};
    data[0] = 1;
    if (kColorMap.find(color1) != kColorMap.end()) {
      data[1] = kColorMap.at(color1);
    } else {
      success = false;
    }
  } else if (subcommand == "blink" && pos != value.npos) {
    data[0] = 2;
    const std::string color1{value.substr(0, pos)};
    if (kColorMap.find(color1) != kColorMap.end()) {
      data[1] = kColorMap.at(color1);
    } else {
      success = false;
    }

    if (success) {
      const std::string substr{value.substr(pos + 1, value.size())};
      const size_t pos2{substr.find(" ")};
      const std::string color2{pos2 == substr.npos ? substr : substr.substr(0, pos2)};
      const std::string frequency_str{substr.substr(pos2 + 1, substr.size() - pos2)};
      const uint8_t frequency{static_cast<uint8_t>(pos2 == substr.npos ? 1 : std::stoi(frequency_str))};
      data[3] = frequency;
      if (kColorMap.find(color2) != kColorMap.end()) {
        data[2] = kColorMap.at(color2);
      } else {
        success = false;
      }
    }
  } else {
    success = false;
    description = "Wrong message: " + command + " " + subcommand + " " + value;
  }

  if (success) {
    frame = DataFrame(SET_LED, data, 4);
  }
  delete[] data;
  data = nullptr;
}

bool Parser::ParseDataFrame(const DataFrame& frame) {
  const uint16_t command = frame.command();
  const uint8_t* data = frame.data();
  const uint16_t length = frame.length();
  switch (command) {
    case CommandFromSensor::MDI: {
      ParseMdiMessage(data, length, laser_scan_);
    } break;
    case CommandFromSensor::SEND_IDENTITY: {
      ParseSendIdentityMessage(data, length);
    } break;
    case CommandFromSensor::SEND_PARAMETERS: {
      ParseSendParametersMessage(data, length, parameters_);
    } break;
    case CommandFromSensor::HEARTBEAT: {
      ParseHeartbeatMessage(data, length, heartbeat_);
    } break;
    case CommandFromSensor::EMERGENCY: {
      ParseEmergencyMessage(data, length, emergency_);
    } break;
    case CommandToSensor::SET_BAUDRATE: {
      ROS_INFO("Set baudrate succeeded: %i", data[0]);
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
      ROS_WARN("Unknown CMD from sensor: %i", command);
      return false;
  }
  return true;
}

void Parser::ParseMdiMessage(const uint8_t*& data, const int& length, sensor_msgs::LaserScan& message) const {
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

  message.header.stamp = ros::Time::now();
  message.header.frame_id = parameters_.header.frame_id;
  message.angle_min = angles::from_degrees(static_cast<float>(parameters_.angle_first) * 1e-2);
  message.angle_max = angles::from_degrees(static_cast<float>(parameters_.angle_last) * 1e-2);
  message.angle_increment = parameters_.mode == 1 ? angles::from_degrees(kHighDensityResolution) : angles::from_degrees(kHighSpeedResolution);
  message.range_min = parameters_.range_min;
  message.range_max = parameters_.range_max;
  message.scan_time = parameters_.mode == 1 ? kHighDensityRefreshPeriod : kHighSpeedRefreshPeriod;
  message.time_increment = message.scan_time / parameters_.number_of_spots;

  message.ranges.clear();
  message.intensities.clear();
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
}

void Parser::ParseSendIdentityMessage(const uint8_t*& data, const int& length) const {
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

void Parser::ParseSendParametersMessage(const uint8_t*& data, const int& length, Parameters& parameters) const {
  const uint32_t verification_code{static_cast<uint32_t>(data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24)};
  const uint16_t charge{static_cast<uint16_t>(data[4] | data[5] << 8)};

  parameters.temperature = data[7];
  parameters.information = data[8];
  parameters.mode = data[9];
  parameters.optimization = data[10];
  parameters.number_of_spots = static_cast<uint16_t>(data[14] | data[15] << 8);
  parameters.angle_first = data[20] | data[21] << 8;
  parameters.angle_last = data[22] | data[23] << 8;
  parameters.counter = data[24];
  parameters.heartbeat_period = data[25];
  parameters.facet = data[26];
  parameters.averaging = data[27];

  ROS_INFO("verification_code: %i", verification_code);
  ROS_INFO("charge: %i", charge);
  ROS_INFO("temperature_enabled: %i", parameters.temperature);
  ROS_INFO("information: %i", parameters.information);
  ROS_INFO("detection_field_mode: %i", parameters.mode);
  ROS_INFO("optimization: %i", parameters.optimization);
  ROS_INFO("number_of_spots: %i", parameters.number_of_spots);
  ROS_INFO("angle_first: %f", static_cast<float>(parameters.angle_first) * 1e-2);
  ROS_INFO("angle_last: %f", static_cast<float>(parameters.angle_last) * 1e-2);
  ROS_INFO("counter_enabled: %i", parameters.counter);
  ROS_INFO("heartbeat_period: %i", parameters.heartbeat_period);
  ROS_INFO("facet_enabled: %i", parameters.facet);
  ROS_INFO("averaging_setting: %i", parameters.averaging);
}

void Parser::ParseHeartbeatMessage(const uint8_t*& data, const int& length, Heartbeat& message) const {
  message.header.stamp = ros::Time::now();
  message.count = static_cast<uint16_t>(data[4] | data[5] << 8);
}

void Parser::ParseEmergencyMessage(const uint8_t*& data, const int& length, Emergency& message) const {
  message.header.stamp = ros::Time::now();
  message.count = static_cast<uint16_t>(data[4] | data[5] << 8);
  message.rs485_error = static_cast<uint16_t>(data[6] | data[7] << 8);
  message.sensor_error = static_cast<uint16_t>(data[8] | data[9] << 8);
  ROS_INFO("Emergency count: %i, error code: %i %i", message.count, message.rs485_error, message.sensor_error);
}

}  // namespace bea_sensors
