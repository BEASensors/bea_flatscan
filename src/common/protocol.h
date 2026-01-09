#pragma once

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>

#include "data_frame.h"
#include "environment.h"

namespace bea_sensors {

constexpr uint16_t kFrameMinimalLength{15};  // 11(SYNC) + 2(CMD) + 2(CHK)
constexpr uint16_t kSyncHeadLength{5};
constexpr uint16_t kSyncTailLength{4};

const uint8_t* const kSyncHead{new uint8_t[kSyncHeadLength]{0xbe, 0xa0, 0x12, 0x34, 0x02}};
const uint8_t* const kSyncTail{new uint8_t[kSyncTailLength]{0x02, 0x00, 0x00, 0x00}};

class Protocol {
 public:
  Protocol(const uint16_t& buffer_size);
  Protocol();
  ~Protocol();

  bool GetLatestDataFrame(DataFrame& frame);
  uint16_t GenerateRawFrame(const uint16_t& command, const uint8_t* data, const uint16_t& length, uint8_t* data_out);
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

  boost::mutex mutex_;

  const uint16_t max_buffer_size_ = 10000;

  Field field_ = Field::SYNC;
  uint16_t data_length_ = 0;
  uint16_t command_ = 0;
  uint16_t checksum_ = 0;
  uint8_t* data_ = nullptr;

  boost::lockfree::spsc_queue<DataFrame, boost::lockfree::capacity<100>> queue_;
};

}  // namespace bea_sensors