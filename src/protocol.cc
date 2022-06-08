#include "protocol.h"

#include <ros/console.h>

constexpr uint16_t polynomial{0x90d9};

namespace bea_sensors {

Protocol::Protocol(const uint16_t& buffer_size) : max_buffer_size_(buffer_size) { data_ = new uint8_t[max_buffer_size_]; }

Protocol::Protocol() { data_ = new uint8_t[max_buffer_size_]; }

Protocol::~Protocol() { delete[] data_; }

uint16_t Protocol::GenerateFrame(const uint16_t& command, const uint8_t* data, const uint16_t& length, uint8_t* data_out) {
  std::copy(kSyncHead, kSyncHead + kSyncHeadLength, data_out);
  uint16_t index{kSyncHeadLength};

  uint16_t frame_length{static_cast<uint16_t>(length + kFrameMinimalLength)};
  data_out[index++] = static_cast<uint8_t>(frame_length & 0xff);  // check the order
  data_out[index++] = static_cast<uint8_t>((frame_length >> 8) & 0xff);

  std::copy(kSyncTail, kSyncTail + kSyncTailLength, data_out + index);
  index += kSyncTailLength;

  data_out[index++] = static_cast<uint8_t>(command & 0xff);  // check the order
  data_out[index++] = static_cast<uint8_t>((command >> 8) & 0xff);

  std::copy(data, data + length, data_out + index);
  index += length;

  uint16_t checksum = CRC16(data_out, index);
  data_out[index++] = static_cast<uint8_t>(checksum & 0xff);  // check the order
  data_out[index++] = static_cast<uint8_t>((checksum >> 8) & 0xff);

  std::cout << "frame:" << std::endl;
  for (uint16_t i = 0; i < index; ++i) {
    std::cout << std::hex << (static_cast<int>(data_out[i]) & 0xff) << " ";
  }
  std::cout << std::endl;

  if (index != frame_length) {
    ROS_ERROR("GenerateFrame error");
    return 0;
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
  static uint16_t index{0};
  if (index < kSyncHeadLength) {
    if (byte == kSyncHead[index]) {
      ++index;
    } else {
      index = 0;
    }
    return false;
  } else if (index < kSyncHeadLength + 2) {
    data_length_ >>= 8;
    data_length_ |= (static_cast<uint16_t>(byte) << 8);
    ++index;
    return false;
  } else if (index < kSyncHeadLength + 2 + kSyncTailLength) {
    if (byte == kSyncTail[index - kSyncHeadLength - 2]) {
      ++index;
    } else {
      index = 0;
    }
    return false;
  } else {
    index = 0;
    data_length_ -= kFrameMinimalLength;
    return true;
  }
}

bool Protocol::ExtractCommand(const uint8_t& byte) {
  static uint16_t index{0};
  if (index < 2) {
    command_ >>= 8;
    command_ |= (static_cast<uint16_t>(byte) << 8);
    ++index;
    return false;
  }

  index = 0;
  return true;
}

bool Protocol::ExtractData(const uint8_t& byte) {
  static uint16_t index{0};
  if (index < data_length_) {
    data_[index] = byte;
    ++index;
    return false;
  }

  index = 0;
  return true;
}

bool Protocol::ExtractChecksum(const uint8_t& byte) {
  static uint16_t index{0};
  if (index < 2) {
    checksum_ >>= 8;
    checksum_ |= (static_cast<uint16_t>(byte) << 8);
    ++index;
    return false;
  }

  index = 0;
  return true;

  const uint16_t frame_length{static_cast<uint16_t>(data_length_ + kFrameMinimalLength - 2)};
  uint8_t* frame = new uint8_t[frame_length];
  for (uint16_t i = 0; i < frame_length; ++i) {
    // TODO: recover the frame, without checksum
  }
  if (checksum_ == CRC16(frame, frame_length)) {
    delete[] frame;
    ROS_INFO("CRC16 check succeeded");
    return true;
  } else {
    delete[] frame;
    ROS_WARN("CRC16 check failed");
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
