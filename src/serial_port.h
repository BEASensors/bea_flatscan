#pragma once

#include <pthread.h>

#include <string>

namespace bea_sensors {

template <class cInstance>
class SerialPort {
 public:
  SerialPort();
  ~SerialPort();

  typedef void (cInstance::*tFunction)(char* data, int length);

  void RegisterCallback(cInstance* instance, tFunction function_ptr);
  int Connect(std::string port, int baud);
  int Close();
  int Write(char* data, int length);

 protected:
  static void* ReceiverRoutine(void* arg);
  int GetBaudrate(int baud);
  void HandleReceivedData(char* data, int length);

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