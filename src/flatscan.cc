#include "flatscan.h"

#include <angles/angles.h>

namespace bea_sensors {

Flatscan::Flatscan(const ros::NodeHandle& nh_) : nh_(nh_) { Initialize(); }

Flatscan::~Flatscan() { com_.Close(); }

void Flatscan::SpinOnce() {
  const ros::Time current_scan_stamp{parser_.laser_scan().header.stamp};
  if (laser_scan_publisher_.getNumSubscribers() > 0 && last_scan_stamp_ < current_scan_stamp) {
    laser_scan_publisher_.publish(parser_.laser_scan());
    last_scan_stamp_ = current_scan_stamp;
  }

  const ros::Time current_heartbeat_stamp{parser_.heartbeat().header.stamp};
  if (heartbeat_publisher_.getNumSubscribers() > 0 && last_heartbeat_stamp_ < current_heartbeat_stamp) {
    heartbeat_publisher_.publish(parser_.heartbeat());
    last_heartbeat_stamp_ = current_heartbeat_stamp;
  }

  const ros::Time current_emergency_stamp{parser_.emergency().header.stamp};
  if (emergency_publisher_.getNumSubscribers() > 0 && last_emergency_stamp_ < current_emergency_stamp) {
    emergency_publisher_.publish(parser_.emergency());
    last_emergency_stamp_ = current_emergency_stamp;
  }
}

bool Flatscan::Initialize() {
  std::string type;
  std::string comm_arg_1;
  int comm_arg_2;
  nh_.param("communication", type, std::string("serial"));
  if (type == "serial") {
    nh_.param("port", comm_arg_1, std::string("/dev/ttyUSB0"));
    nh_.param("baudrate", comm_arg_2, 921600);
  } else if (type == "ethernet") {
    nh_.param("ip", comm_arg_1, std::string("192.168.1.199"));
    nh_.param("port", comm_arg_2, 20108);
  }

  std::string scan_topic, heartbeat_topic, emergency_topic;
  nh_.param("scan_topic", scan_topic, std::string("/scan"));
  nh_.param("heartbeat_topic", heartbeat_topic, std::string("/heartbeat"));
  nh_.param("emergency_topic", emergency_topic, std::string("/emergency"));

  Parameters parameters;
  nh_.param("scan_frame_id", parameters.header.frame_id, std::string("laser_link"));
  nh_.param("min_range", parameters.range_min, static_cast<float>(0.));
  nh_.param("max_range", parameters.range_max, static_cast<float>(8.));

  int temp;
  nh_.param("enable_temperature", temp, static_cast<int>(1));
  parameters.temperature = static_cast<uint8_t>(temp);
  nh_.param("information_in_mdi", temp, static_cast<int>(0));
  parameters.information = static_cast<uint8_t>(temp);
  std::string detection_field_mode;
  nh_.param("detection_field_mode", detection_field_mode, std::string("HD"));
  if (detection_field_mode == "HD") {
    ROS_INFO("Flatscan works in HD mode, loading HD configurations");
    parameters.mode = 1;
    parameters.number_of_spots = 400;
  } else if (detection_field_mode == "HS") {
    ROS_INFO("Flatscan works in HS mode, loading HS configurations");
    parameters.mode = 0;
    parameters.number_of_spots = 100;
  } else {
    ROS_WARN("Flatscan works in %s(UNKNOWN) mode, loading HD configurations by default", detection_field_mode.c_str());
    parameters.mode = 1;
    parameters.number_of_spots = 400;
  }
  nh_.param("optimization", temp, static_cast<int>(0));
  parameters.optimization = static_cast<uint8_t>(temp);
  float angle_first, angle_last;
  nh_.param("angle_first", angle_first, static_cast<float>(0.));
  nh_.param("angle_last", angle_last, static_cast<float>(108.));
  parameters.angle_first = static_cast<uint16_t>(angle_first * 1e2);
  parameters.angle_last = static_cast<uint16_t>(angle_last * 1e2);
  nh_.param("enable_counter", temp, static_cast<int>(1));
  parameters.counter = static_cast<uint8_t>(temp);
  nh_.param("heartbeat_period", temp, static_cast<int>(5));
  parameters.heartbeat_period = static_cast<uint8_t>(temp);
  nh_.param("enable_facet", temp, static_cast<int>(1));
  parameters.facet = static_cast<uint8_t>(temp);
  nh_.param("averaging_setting", temp, static_cast<int>(0));
  parameters.averaging = static_cast<uint8_t>(temp);

  laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 10, this);
  heartbeat_publisher_ = nh_.advertise<Heartbeat>(heartbeat_topic, 10, this);
  emergency_publisher_ = nh_.advertise<Emergency>(emergency_topic, 10, this);
  configuration_server_ = nh_.advertiseService("configure", &Flatscan::HandleConfiguration, this);

  com_.RegisterCallback(this, &Flatscan::HandleReceivedData);
  com_.Connect(type, comm_arg_1, comm_arg_2);

  InitializeConfiguration(parameters);
  return true;
}

bool Flatscan::InitializeConfiguration(const Parameters& parameters) {
  parser_.Initialize(parameters);
  Configure srv;
  srv.request.command = "set_parameters";
  srv.request.subcommand = "";
  HandleConfiguration(srv.request, srv.response);
  return srv.response.success;
}

bool Flatscan::HandleConfiguration(Configure::Request& req, Configure::Response& res) {
  DataFrame frame;
  bool success{false};
  if (!parser_.GenerateDataFrame(req.command, req.subcommand, req.value, success, res.description, frame)) {
    res.success = success;
    return true;
  }
  res.success = success;
  SendMessage(frame.command(), frame.length(), frame.data());
  return true;
}

void Flatscan::SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data) {
  std::unique_lock<std::mutex> lock(mutex_);
  message_sent_ = false;
  lock.unlock();
  uint8_t data_out[data_length + kFrameMinimalLength];
  uint16_t length = protocol_.GenerateRawFrame(command, data, data_length, data_out);
  if (length < kFrameMinimalLength) {
    return;
  }

  uint8_t retries{0};
  while (!message_sent_ && retries < 10) {
    com_.Write((char*)data_out, length);
    ++retries;
    sleep(1);
  }

  if (!message_sent_) {
    ROS_ERROR("send message failed");
  }
}

void Flatscan::HandleReceivedData(char* data, int length) {
  for (int i = 0; i < length; ++i) {
    if (protocol_.InsertByte(data[i]) < 0) {
      continue;
    }

    DataFrame frame;
    if (!protocol_.GetLatestDataFrame(frame)) {
      ROS_ERROR("Get received frame error");
      continue;
    }

    if (!parser_.ParseDataFrame(frame)) {
      ROS_ERROR("Parse frame failed");
      continue;
    }

    if (frame.command() != MDI) {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
    }
  }
}

}  // namespace bea_sensors
