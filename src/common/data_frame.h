#pragma once

namespace bea_sensors {

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

  const uint16_t& command() const { return command_; }
  const uint16_t& length() const { return length_; }
  const uint8_t* data() const { return data_; }

 private:
  uint16_t command_;
  uint16_t length_;
  uint8_t* data_;
};

}  // namespace bea_sensors