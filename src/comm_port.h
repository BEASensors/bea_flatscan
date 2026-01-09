#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

namespace bea_sensors {

template <class cInstance>
class CommPort {
 public:
  CommPort();
  ~CommPort();

  typedef void (cInstance::*tFunction)(char* data, int length);
  typedef void (cInstance::*tOnConnected)();

  void RegisterCallback(cInstance* instance, tFunction function_ptr);
  void RegisterConnectedCallback(cInstance* instance, tOnConnected on_connected_ptr);
  int Connect(std::string type, std::string port, int baud);
  int Close();
  int Write(char* data, int length);

 protected:
  static void* ReceiverRoutine(void* arg);
  static void* SerialReceiverRoutine(CommPort* port);
  static void* EthernetReceiverRoutine(CommPort* port);
  int GetBaudrate(int baud);
  void HandleReceivedData(char* data, int length);

 protected:
  bool is_running_ = false;
  bool is_connected_ = false;

  pthread_t receiving_thread_;
  pthread_mutex_t write_mutex_;

  std::string type_;
  std::string port_;
  int serial_fd_ = 0;
  int baudrate_;

  cInstance* instance_;
  tFunction function_ptr_ = 0;
  tOnConnected on_connected_ptr_ = 0;
};

template <class cInstance>
CommPort<cInstance>::CommPort() {
  pthread_mutex_init(&write_mutex_, nullptr);
}

template <class cInstance>
CommPort<cInstance>::~CommPort() {
  pthread_mutex_destroy(&write_mutex_);
}

template <class cInstance>
void CommPort<cInstance>::RegisterCallback(cInstance* instance, CommPort<cInstance>::tFunction function_ptr) {
  instance_ = instance;
  function_ptr_ = function_ptr;
  RCLCPP_INFO(rclcpp::get_logger("bea_sensors"), "Callback function registered");
}

template <class cInstance>
void CommPort<cInstance>::RegisterConnectedCallback(cInstance* instance, CommPort<cInstance>::tOnConnected on_connected_ptr) {
  instance_ = instance;
  on_connected_ptr_ = on_connected_ptr;
}

template <class cInstance>
int CommPort<cInstance>::Connect(std::string type, std::string port, int baud) {
  type_ = type;
  port_ = port;
  baudrate_ = baud;
  is_running_ = true;

  RCLCPP_INFO(rclcpp::get_logger("bea_sensors"), "Creating data receiving thread");
  if (pthread_create(&receiving_thread_, nullptr, ReceiverRoutine, (void*)this)) {
    return -1;
  }

  return 1;
}

template <class cInstance>
int CommPort<cInstance>::Close() {
  is_running_ = false;
  pthread_join(receiving_thread_, nullptr);
  close(serial_fd_);
  return 1;
}

template <class cInstance>
int CommPort<cInstance>::Write(char* data, int length) {
  if (!is_connected_) {
    return 0;
  }
  int res = 0;
  pthread_mutex_lock(&write_mutex_);
  if (type_ == "serial") {
    res = write(serial_fd_, data, length);
  } else if (type_ == "ethernet") {
    res = send(serial_fd_, data, length, 0);
  }
  pthread_mutex_unlock(&write_mutex_);
  return res;
}

template <class cInstance>
void* CommPort<cInstance>::ReceiverRoutine(void* arg) {
  CommPort* port = (CommPort*)arg;
  if (port->type_ == "serial") {
    return SerialReceiverRoutine(port);
  } else if (port->type_ == "ethernet") {
    return EthernetReceiverRoutine(port);
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("bea_sensors"), "Unknown communication type, please check!");
    return nullptr;
  }
}

template <class cInstance>
void* CommPort<cInstance>::SerialReceiverRoutine(CommPort* port) {
  while (port->is_running_) {
    // Try to reconnect
    int retries{0};
    while (port->is_running_) {
      port->serial_fd_ = open(port->port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (port->serial_fd_ < 0) {
        ++retries;
        if (retries > 10) {
          RCLCPP_ERROR(rclcpp::get_logger("bea_sensors"), "Cannot open %s, please check", port->port_.c_str());
          return nullptr;
        }
        RCLCPP_WARN(rclcpp::get_logger("bea_sensors"), "Open %s failed, tried %d times, try again..", port->port_.c_str(), retries);
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
      RCLCPP_INFO(rclcpp::get_logger("bea_sensors"), "Open %s successfully (baudrate: %d), let's rock!", port->port_.c_str(), port->baudrate_);
      if (port->on_connected_ptr_) {
        (port->instance_->*port->on_connected_ptr_)();
      }
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
  RCLCPP_INFO(rclcpp::get_logger("bea_sensors"), "Port %s closed", port->port_.c_str());
  return nullptr;
}

template <class cInstance>
void* CommPort<cInstance>::EthernetReceiverRoutine(CommPort* port) {
  struct sockaddr_in server;

  while (port->is_running_) {
    // Create socket
    port->serial_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (port->serial_fd_ == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("bea_sensors"), "Cannot create socket");
      port->is_connected_ = false;
      sleep(1);
      continue;
    }

    // Socket options: keepalive, recv timeout, nodelay
    int yes = 1;
    setsockopt(port->serial_fd_, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));

    struct timeval rcv_timeout;
    rcv_timeout.tv_sec = 1;  // 1s receive timeout
    rcv_timeout.tv_usec = 0;
    setsockopt(port->serial_fd_, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));

    setsockopt(port->serial_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    // Build server address (port_->port_ holds IP, baudrate_ holds TCP port)
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(port->baudrate_);
    server.sin_addr.s_addr = inet_addr(port->port_.c_str());

    // Connect
    if (connect(port->serial_fd_, (struct sockaddr*)&server, sizeof(server)) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("bea_sensors"), "Connection failed");
      port->is_connected_ = false;
      close(port->serial_fd_);
      sleep(1);
      continue;  // retry
    }

    port->is_connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("bea_sensors"), "Connected to server");
    if (port->on_connected_ptr_) {
      (port->instance_->*port->on_connected_ptr_)();
    }

    // Receive loop
    while (port->is_running_) {
      char buf[1024];
      int size = recv(port->serial_fd_, buf, sizeof(buf), 0);
      if (size > 0) {
        port->HandleReceivedData(buf, size);
        continue;
      }

      // size == 0: peer closed; size < 0: timeout or error
      if (size == 0) {
        RCLCPP_WARN(rclcpp::get_logger("bea_sensors"), "Server closed connection, will reconnect");
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // recv timeout - treat as inactivity, trigger reconnect to recover after device power cycle
          RCLCPP_WARN(rclcpp::get_logger("bea_sensors"), "No data (timeout), reconnecting...");
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("bea_sensors"), "Receive error (%d), reconnecting...", errno);
        }
      }
      break;  // break inner loop to reconnect
    }

    // Clean up and prepare to reconnect
    port->is_connected_ = false;
    close(port->serial_fd_);
    // loop continues to reconnect while is_running_
  }

  RCLCPP_INFO(rclcpp::get_logger("bea_sensors"), "Ethernet receiver stopped");
  return nullptr;
}


template <class cInstance>
int CommPort<cInstance>::GetBaudrate(int baud) {
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
void CommPort<cInstance>::HandleReceivedData(char* data, int length) {
  (instance_->*function_ptr_)(data, length);
}

}  // namespace bea_sensors