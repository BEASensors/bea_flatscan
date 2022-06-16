#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <ros/console.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>

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

template <class cInstance>
SerialPort<cInstance>::SerialPort() {
  pthread_mutex_init(&write_mutex_, nullptr);
}

template <class cInstance>
SerialPort<cInstance>::~SerialPort() {
  pthread_mutex_destroy(&write_mutex_);
}

template <class cInstance>
void SerialPort<cInstance>::RegisterCallback(cInstance* instance, SerialPort<cInstance>::tFunction function_ptr) {
  instance_ = instance;
  function_ptr_ = function_ptr;
  ROS_INFO("Callback function registered");
}

template <class cInstance>
int SerialPort<cInstance>::Connect(std::string port, int baud) {
  port_ = port;
  baudrate_ = baud;
  is_running_ = true;

  ROS_INFO("Creating data receiving thread");
  if (pthread_create(&receiving_thread_, nullptr, ReceiverRoutine, (void*)this)) {
    return -1;
  }

  return 1;
}

template <class cInstance>
int SerialPort<cInstance>::Close() {
  is_running_ = false;
  pthread_join(receiving_thread_, nullptr);
  close(serial_fd_);
  return 1;
}

template <class cInstance>
int SerialPort<cInstance>::Write(char* data, int length) {
  if (!is_connected_) {
    return 0;
  }
  pthread_mutex_lock(&write_mutex_);
  int res = write(serial_fd_, data, length);
  pthread_mutex_unlock(&write_mutex_);
  return res;
}

template <class cInstance>
void* SerialPort<cInstance>::ReceiverRoutine(void* arg) {
  SerialPort* port = (SerialPort*)arg;
  while (port->is_running_) {
    // Try to reconnect
    int retries{0};
    while (port->is_running_) {
      port->serial_fd_ = open(port->port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (port->serial_fd_ < 0) {
        ++retries;
        if (retries > 10) {
          ROS_ERROR("Cannot open %s, please check", port->port_.c_str());
          return nullptr;
        }
        ROS_WARN("Open %s failed, tried %d times, try again..", port->port_.c_str(), retries);
        sleep(1);
        continue;
      }

      struct termios oldtio, newtio;
      tcgetattr(port->serial_fd_, &oldtio);  // save current port settings

      int baudrate = port->GetBaudrate(port->baudrate_);

      bzero(&newtio, sizeof(newtio));
      newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
      newtio.c_iflag = IGNPAR;
      newtio.c_oflag = 0;
      newtio.c_lflag = 0;
      newtio.c_cc[VMIN] = 1;
      newtio.c_cc[VTIME] = 0;
      tcflush(port->serial_fd_, TCIFLUSH);
      tcsetattr(port->serial_fd_, TCSANOW, &newtio);

      port->is_connected_ = true;
      ROS_INFO("Open %s successfully (baudrate: %d), let's rock!", port->port_.c_str(), port->baudrate_);
      break;
    }

    struct pollfd fds[1];
    fds[0].fd = port->serial_fd_;
    fds[0].events = POLLIN | POLLERR | POLLHUP;

    while (port->is_running_) {
      int res = poll(fds, 1, 100000);
      if (res < 0) {
        continue;
      }

      if (fds[0].revents & POLLIN) {
        char buf[255];
        int size = read(port->serial_fd_, buf, 255);
        if (size > 0) {
          port->HandleReceivedData(buf, size);
        }
      }
      if (fds[0].revents & POLLERR) {
        break;
      }
      if (fds[0].revents & POLLHUP) {
        break;
      }
    }
    port->is_connected_ = false;
  }

  close(port->serial_fd_);
  ROS_INFO("Port %s closed", port->port_.c_str());
  return nullptr;
}

template <class cInstance>
int SerialPort<cInstance>::GetBaudrate(int baud) {
  switch (baud) {
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B921600;
  }
}

template <class cInstance>
void SerialPort<cInstance>::HandleReceivedData(char* data, int length) {
  (instance_->*function_ptr_)(data, length);
}

}  // namespace bea_sensors