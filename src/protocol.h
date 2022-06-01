#pragma once

#include <boost/lockfree/spsc_queue.hpp>

namespace bea_sensors {

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

  DataFrame(int command, uint8_t* data, int length) {
    data_ = new uint8_t[length];
    length_ = length;
    command_ = command;
    std::copy(data, data + length, data_);
  }

  ~DataFrame() {
    if (data_) {
      delete (data_);
      data_ = nullptr;
    }
  }

  const uint16_t command() { return command_; }
  const uint16_t length() { return length_; }
  const uint8_t* data() { return data_; }

 private:
  uint16_t command_;
  uint16_t length_;
  uint8_t* data_;
};

class Protocol {
 public:
  Protocol(int buffer_size);
  Protocol();
  ~Protocol();

  bool GetReceivedFrame(DataFrame& frame) { return queue_.pop(frame); }
  int GenerateFrame(const uint16_t& command, const uint8_t* data, const uint16_t& length, uint8_t* data_out);
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
  const std::array<uint8_t, 5> sync_head_{0xbe, 0xa0, 0x12, 0x34, 0x02};
  const std::array<uint8_t, 4> sync_tail_{0x02, 0x00, 0x00, 0x00};

  Field field_ = Field::SYNC;
  uint16_t data_length_ = 0;
  uint16_t command_ = 0;
  uint16_t checksum_ = 0;
  uint8_t* data_;

  boost::lockfree::spsc_queue<DataFrame, boost::lockfree::capacity<100> > queue_;
};

}  // namespace bea_sensors