#pragma once

#include <boost/lockfree/spsc_queue.hpp>
#include <unordered_map>

namespace bea_sensors {

constexpr uint16_t kFrameMinimalLength{15};  // 11(SYNC) + 2(CMD) + 2(CHK)
constexpr uint16_t kSyncHeadLength{5};
constexpr uint16_t kSyncTailLength{4};

const uint8_t* const kSyncHead{new uint8_t[kSyncHeadLength]{0xbe, 0xa0, 0x12, 0x34, 0x02}};
const uint8_t* const kSyncTail{new uint8_t[kSyncTailLength]{0x02, 0x00, 0x00, 0x00}};

const std::unordered_map<std::string, uint8_t> kParameterMap{/*{"temperature", 1},*/ {"information", 2}, /*{"mode", 3},        {"optimization", 4},*/
                                                             /*{"spots", 8},       {"angle_first", 14}, {"angle_last", 15}, {"averaging", 21}*/};
const std::unordered_map<std::string, uint8_t> kInformationParameterMap{{"distances", 0}, {"remissions", 1}, {"distances and remissions", 2}};
const std::unordered_map<std::string, uint8_t> kOptimizationParameterMap;
const std::unordered_map<std::string, uint8_t> kAveragingParameterMap;
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

class DataFrame {
 public:
  DataFrame() { data_ = nullptr; }

  DataFrame(uint16_t command, uint8_t* data, uint16_t length) {
    data_ = new uint8_t[length];
    length_ = length;
    command_ = command;
    std::copy(data, data + length, data_);
  }

  DataFrame(const DataFrame& other) {
    data_ = new uint8_t[other.length_];
    length_ = other.length_;
    command_ = other.command_;
    std::copy(other.data_, other.data_ + other.length_, data_);
  }

  ~DataFrame() {
    if (data_) {
      delete[] data_;
      data_ = nullptr;
    }
  }

  DataFrame& operator=(const DataFrame& other) {
    if (this == &other) {
      return *this;
    }

    if (data_) {
      delete[] data_;
      data_ = nullptr;
    }

    data_ = new uint8_t[other.length_];
    length_ = other.length_;
    command_ = other.command_;
    std::copy(other.data_, other.data_ + other.length_, data_);

    return *this;
  }

  const uint16_t& command() { return command_; }
  const uint16_t& length() { return length_; }
  const uint8_t* data() { return data_; }

 private:
  uint16_t command_;
  uint16_t length_;
  uint8_t* data_;
};

class Protocol {
 public:
  Protocol(const uint16_t& buffer_size);
  Protocol();
  ~Protocol();

  bool GetLatestFrame(DataFrame& frame) { return queue_.pop(frame); }
  uint16_t GenerateFrame(const uint16_t& command, const uint8_t* data, const uint16_t& length, uint8_t* data_out);
  int InsertByte(const uint8_t& byte);

 private:
  bool ExtractSync(const uint8_t& byte);
  bool ExtractCommand(const uint8_t& byte);
  bool ExtractData(const uint8_t& byte);
  bool ExtractChecksum(const uint8_t& byte);
  uint16_t CRC16(uint8_t* buf, uint16_t cnt);

 private:
  enum Field {
    SYNC = 0,
    CMD,
    DATA,
    CHK,
  };

  const uint16_t max_buffer_size_ = 10000;

  Field field_ = Field::SYNC;
  uint16_t data_length_ = 0;
  uint16_t command_ = 0;
  uint16_t checksum_ = 0;
  uint8_t* data_ = nullptr;

  boost::lockfree::spsc_queue<DataFrame, boost::lockfree::capacity<100>> queue_;
};

}  // namespace bea_sensors