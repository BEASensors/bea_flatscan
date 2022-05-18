#pragma once

#include <pthread.h>

#include <string>

namespace bea_sensors {

template <class cInstance>
class RtSerialPort {
 public:
  RtSerialPort();
  ~RtSerialPort();

  typedef void (cInstance::*tFunction)(char* data, int length);

  void registerCallback(cInstance* instance, tFunction function_ptr);
  int connect(std::string port, int baud);
  int close();
  int write(char* data, int length);

 protected:
  static void* receiver_thread(void* arg);
  int getBaudrate(int baud);
  void handleReceivedData(char* data, int length);

 protected:
  bool is_running_ = false;
  bool is_connected_ = false;

  pthread_t receiving_thread_;
  pthread_mutex_t write_mutex_;

  std::string port_;
  int serial_fd_ = 0;
  int baudrate_;

  cInstance* instance_;
  tFunction function_ptr_ = 0;
};

}  // namespace bea_sensors