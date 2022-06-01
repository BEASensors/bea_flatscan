#include "protocol.h"

#include <ros/console.h>

#define HEADER_SIZE 4
#define MSG_SIZE (HEADER_SIZE + 1 + 2 + 2)  // header(4) + data(1)+cs(2) + end_sync(2)= 9

constexpr uint16_t polynomial{0x90d9};

namespace bea_sensors {

Protocol::Protocol(int buffer_size) : max_buffer_size_(buffer_size) { data_ = new uint8_t[max_buffer_size_]; }

Protocol::Protocol() { data_ = new uint8_t[max_buffer_size_]; }

Protocol::~Protocol() { delete data_; }

int Protocol::GenerateFrame(const uint16_t& command, const uint8_t* data, const uint16_t& length, uint8_t* data_out) {
  size_t index{0};
  for (size_t i = 0; i < sync_head_.size(); ++i) {
    data_out[index++] = sync_head_.at(i);
  }

  uint16_t frame_length{length + 15};
  data_out[index++] = static_cast<uint8_t>(frame_length & 0xff);  // check the order
  data_out[index++] = static_cast<uint8_t>((frame_length >> 8) & 0xff);

  for (size_t i = 0; i < sync_tail_.size(); ++i) {
    data_out[index++] = sync_tail_.at(i);
  }

  data_out[index++] = static_cast<uint8_t>(command & 0xff);  // check the order
  data_out[index++] = static_cast<uint8_t>((command >> 8) & 0xff);

  for (uint16_t i = 0; i < length; ++i) {
    data_out[index++] = data[i];
  }

  uint16_t checksum = CRC16(data_out, index);
  data_out[index++] = static_cast<uint8_t>(checksum & 0xff);  // check the order
  data_out[index++] = static_cast<uint8_t>((checksum >> 8) & 0xff);

  if (index != frame_length) {
    ROS_ERROR("GenerateFrame error");
  }

  return index;
}

int Protocol::InsertByte(const uint8_t& byte) {
  switch (field_) {
    case Field::SYNC:
      if (ExtractSync(byte)) {
        field_ = Field::CMD;
      } else {
        return -1;
      }
    case Field::CMD:
      if (ExtractCommand(byte)) {
        field_ = Field::DATA;
      } else {
        return -2;
      }
    case Field::DATA:
      if (ExtractData(byte)) {
        field_ = Field::CHK;
      } else {
        return -3;
      }
    case Field::CHK:
      if (ExtractChecksum(byte)) {
        field_ = Field::SYNC;
        DataFrame frame(command_, data_, data_length_);
        queue_.push(frame);
        return 1;
      } else {
        return -4;
      }
    default:
      return -5;
  }
}

bool Protocol::ExtractSync(const uint8_t& byte) {
  static const size_t head_size{sync_head_.size()};
  static const size_t length_size{head_size + 2};
  static const size_t tail_size{length_size + sync_tail_.size()};
  static size_t index{0};
  if (index < head_size) {
    if (byte == sync_head_.at(index)) {
      ++index;
    } else {
      index = 0;
    }
    return false;
  } else if (index < length_size) {
    data_length_ <<= 8;
    data_length_ |= static_cast<uint16_t>(byte);  // check the order
    ++index;
    return false;
  } else if (index < tail_size) {
    if (byte == sync_tail_.at(index - length_size)) {
      ++index;
    } else {
      index = 0;
    }
    return false;
  } else {
    index = 0;
    data_length_ -= 15;  // 11(SYNC) + 2(CMD) + 2(CHK)
    ROS_INFO("Received new frame with data length %d", data_length_);
    return true;
  }
}

bool Protocol::ExtractCommand(const uint8_t& byte) {
  static size_t index{0};
  if (index < 2) {
    command_ <<= 8;
    command_ |= static_cast<uint16_t>(byte);  // check the order
    ++index;
    return false;
  }

  index = 0;
  ROS_INFO("Received valid command: %x", command_);
  return true;
}

bool Protocol::ExtractData(const uint8_t& byte) {
  static size_t index{0};
  if (index < data_length_) {
    data_[index] = byte;
    ++index;
    return false;
  }

  index = 0;
  return true;
}

bool Protocol::ExtractChecksum(const uint8_t& byte) {
  static size_t index{0};
  if (index < 2) {
    checksum_ <<= 8;
    checksum_ |= static_cast<uint16_t>(byte);  // check the order
    ++index;
    return false;
  }

  index = 0;
  const size_t frame_length{static_cast<size_t>(data_length_) + 13};
  uint8_t* frame = new uint8_t[frame_length];
  for (size_t i = 0; i < frame_length; ++i) {
    // TODO: recover the frame, without checksum
  }
  if (checksum_ == CRC16(frame, frame_length)) {
    delete frame;
    return true;
  } else {
    delete frame;
    return false;
  }
}

uint16_t Protocol::CRC16(uint8_t* buf, uint16_t cnt) {
  uint16_t crc = 0; /* CRC value is 16bit */
  uint16_t i, j;
  for (i = 0; i < cnt; ++i) {
    crc ^= (uint16_t)(buf[i] << 8); /* move byte into MSB of 16bit CRC */
    for (j = 0; j < 8; ++j) {
      if ((crc & 0x8000) != 0) { /* test for MSB = bit 15 */
        crc = (uint16_t)((crc << 1) ^ polynomial);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

}  // namespace bea_sensors
