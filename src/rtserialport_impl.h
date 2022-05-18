#pragma once

#include <fcntl.h>
#include <ros/console.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>

#include "rtserialport.h"

namespace bea_sensors {

template <class cInstance>
RtSerialPort<cInstance>::RtSerialPort() {
  pthread_mutex_init(&write_mutex_, nullptr);
}

template <class cInstance>
RtSerialPort<cInstance>::~RtSerialPort() {
  pthread_mutex_destroy(&write_mutex_);
}

template <class cInstance>
void RtSerialPort<cInstance>::registerCallback(cInstance* instance, RtSerialPort<cInstance>::tFunction function_ptr) {
  instance_ = instance;
  function_ptr_ = function_ptr;
}

template <class cInstance>
int RtSerialPort<cInstance>::connect(std::string port, int baud) {
  port_ = port;
  baudrate_ = baud;
  is_running_ = true;

  /* Start receiving thread */
  if (pthread_create(&receiving_thread_, nullptr, receiver_thread, (void*)this)) {
    return -1;
  }

  return 1;
}

template <class cInstance>
int RtSerialPort<cInstance>::close() {
  is_running_ = false;
  pthread_join(receiving_thread_, nullptr);
  ::close(serial_fd_);
  return 1;
}

template <class cInstance>
int RtSerialPort<cInstance>::write(char* data, int length) {
  if (!is_connected_) {
    return 0;
  }
  pthread_mutex_lock(&write_mutex_);
  int res = ::write(serial_fd_, data, length);
  pthread_mutex_unlock(&write_mutex_);
  return res;
}

template <class cInstance>
void* RtSerialPort<cInstance>::receiver_thread(void* arg) {
  RtSerialPort* port = (RtSerialPort*)arg;
  while (port->is_running_) {
    // Try to reconnect
    int retries{0};
    while (port->is_running_) {
      port->serial_fd_ = open(port->port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (port->serial_fd_ < 0) {
        ++retries;
        if (retries > 30) {
          ROS_ERROR("Cannot open %s, please check", port->port_.c_str());
          return nullptr;
        }
        ROS_WARN("Open %s failed, tried %d times, try again..", port->port_.c_str(), retries);
        sleep(1);
        continue;
      }

      struct termios oldtio, newtio;
      tcgetattr(port->serial_fd_, &oldtio);  // save current port settings

      int baudrate = port->getBaudrate(port->baudrate_);

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
          port->handleReceivedData(buf, size);
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

  ::close(port->serial_fd_);
  return nullptr;
}

template <class cInstance>
int RtSerialPort<cInstance>::getBaudrate(int baud) {
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
      return B57600;
  }
}

template <class cInstance>
void RtSerialPort<cInstance>::handleReceivedData(char* data, int length) {
  (instance_->*function_ptr_)(data, length);
}

}  // namespace bea_sensors
