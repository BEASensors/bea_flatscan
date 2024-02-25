#pragma once

#include <sensor_msgs/LaserScan.h>

#include <mutex>
#include <unordered_map>

#include "bea_sensors/Configure.h"
#include "bea_sensors/Emergency.h"
#include "bea_sensors/Heartbeat.h"
#include "bea_sensors/Parameters.h"
#include "data_frame.h"

namespace bea_sensors {

constexpr float kHighDensityResolution{0.18 + 0.09};
constexpr float kHighDensityRefreshPeriod{43 * 1e-3};
constexpr float kHighSpeedResolution{0.74 + 0.09};
constexpr float kHighSpeedRefreshPeriod{10.75 * 1e-3};

const std::unordered_map<std::string, uint8_t> kParameterMap{{"temperature", 1}, {"information", 2},  {"mode", 3},        {"optimization", 4},
                                                             {"spots", 8},       {"angle_first", 14}, {"angle_last", 16}, {"counter", 18},
                                                             {"heartbeat", 19},  {"facet", 20},       {"averaging", 21}};
const std::unordered_map<std::string, uint8_t> kBaudrateMap{{"57600", 0}, {"115200", 1}, {"230400", 2}, {"460800", 3}, {"921600", 4},
                                                            {"0", 0},     {"1", 1},      {"2", 2},      {"3", 3},      {"4", 4}};
const std::unordered_map<std::string, uint8_t> kColorMap{{"off", 0}, {"red", 1}, {"green", 2}, {"orange", 3}};

enum CommandToSensor {
  SET_BAUDRATE = (0xc3 << 8 | 0x51),
  GET_MEASUREMENTS = (0xc3 << 8 | 0x5b),
  GET_IDENTITY = (0xc3 << 8 | 0x5a),
  GET_EMERGENCY = (0xc3 << 8 | 0x6e),
  GET_PARAMETERS = (0xc3 << 8 | 0x54),
  SET_PARAMETERS = (0xc3 << 8 | 0x53),
  STORE_PARAMETERS = (0xc3 << 8 | 0x55),
  RESET_MDI_COUNTER = (0xc3 << 8 | 0x5e),
  RESET_HEARTBEAT_COUNTER = (0xc3 << 8 | 0x5f),
  RESET_EMERGENCY_COUNTER = (0xc3 << 8 | 0x61),
  SET_LED = (0xc3 << 8 | 0x78),
};

enum CommandFromSensor {
  MDI = (0xc3 << 8 | 0x5b),
  SEND_IDENTITY = (0xc3 << 8 | 0x5a),
  SEND_PARAMETERS = (0xc3 << 8 | 0x54),
  HEARTBEAT = (0xc3 << 8 | 0x64),
  EMERGENCY = (0xc3 << 8 | 0x6e),
};

class Parser {
 public:
  Parser();
  ~Parser();

  void Initialize(const Parameters& parameters = Parameters()) { parameters_ = parameters; }
  bool GenerateDataFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success, std::string& description,
                         DataFrame& frame);
  bool ParseDataFrame(const DataFrame& frame);

  const sensor_msgs::LaserScan& laser_scan() {
    std::lock_guard<std::mutex> lock(laser_mutex_);
    return laser_scan_;
  }
  const Heartbeat& heartbeat() {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    return heartbeat_;
  }
  const Emergency& emergency() {
    std::lock_guard<std::mutex> lock(emergency_mutex_);
    return emergency_;
  }
  const Parameters& parameters() {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    return parameters_;
  }

 private:
  void GenerateSetBaudrateFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                std::string& description, DataFrame& frame) const;
  void GenerateGetMeasurementsFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                    std::string& description, DataFrame& frame) const;
  void GenerateSetParametersFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                                  std::string& description, DataFrame& frame) const;
  void GenerateSetLedFrame(const std::string& command, const std::string& subcommand, const std::string& value, bool& success,
                           std::string& description, DataFrame& frame) const;

  void ParseMdiMessage(const uint8_t*& data, const int& length, sensor_msgs::LaserScan& message) const;
  void ParseSendIdentityMessage(const uint8_t*& data, const int& length) const;
  void ParseSendParametersMessage(const uint8_t*& data, const int& length, Parameters& parameters) const;
  void ParseHeartbeatMessage(const uint8_t*& data, const int& length, Heartbeat& message) const;
  void ParseEmergencyMessage(const uint8_t*& data, const int& length, Emergency& message) const;

 private:
  std::mutex laser_mutex_;
  std::mutex heartbeat_mutex_;
  std::mutex emergency_mutex_;
  std::mutex parameters_mutex_;

  sensor_msgs::LaserScan laser_scan_;
  Heartbeat heartbeat_;
  Emergency emergency_;
  Parameters parameters_;
};

}  // namespace bea_sensors