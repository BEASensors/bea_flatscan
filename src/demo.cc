#include <angles/angles.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "flat_scan.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bea_sensor_driver");
  ros::NodeHandle nh("~");

  std::string port;
  nh.param("port", port, std::string("/dev/ttyUSB0"));

  int baudrate;
  nh.param("baudrate", baudrate, 921600);

  std::string detection_field_mode;
  nh.param("detection_field_mode", detection_field_mode, std::string("HD"));

  int number_of_spots;
  float resolution, refresh_period;
  if (detection_field_mode == "HD") {
    ROS_INFO("FlatScan works in HD mode, loading HD configurations");
    resolution = 0.18 + 0.09;    // in degrees
    refresh_period = 43 * 1e-3;  // in seconds
    number_of_spots = 400;
  } else if (detection_field_mode == "HS") {
    ROS_INFO("FlatScan works in HS mode, loading HS configurations");
    resolution = 0.74 + 0.09;       // in degrees
    refresh_period = 10.75 * 1e-3;  // in seconds
    number_of_spots = 100;
  } else {
    ROS_WARN("FlatScan works in UNKNOWN mode, loading HD configurations by default");
    resolution = 0.18 + 0.09;    // in degrees
    refresh_period = 43 * 1e-3;  // in seconds
    number_of_spots = 400;
  }

  float first_angle, last_angle;
  nh.param("first_angle", first_angle, static_cast<float>(0.));
  nh.param("last_angle", last_angle, static_cast<float>(108.));

  float min_range, max_range;
  nh.param("min_range", min_range, static_cast<float>(0.));
  nh.param("max_range", max_range, static_cast<float>(8.));

  std::string frame_id;
  nh.param("frame_id", frame_id, std::string("laser_link"));

  std::string topic_name;
  nh.param("topic_name", topic_name, std::string("/scan"));

  float loop_rate;
  nh.param("loop_rate", loop_rate, static_cast<float>(10.));

  bea_sensors::FlatScan flat_scan(port, baudrate);

  ros::Publisher laser_scan_publisher = nh.advertise<sensor_msgs::LaserScan>(topic_name, 1);
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = frame_id;
  scan.angle_min = angles::from_degrees(first_angle);
  scan.angle_max = angles::from_degrees(last_angle);
  scan.angle_increment = angles::from_degrees(resolution);
  scan.range_min = min_range;
  scan.range_max = max_range;
  scan.scan_time = refresh_period;
  scan.time_increment = refresh_period / number_of_spots;

  ros::Rate rate(loop_rate);
  while (ros::ok()) {
    const bea_sensors::LaserData& data{flat_scan.laser_data()};
    if (data.length == 0) {
      rate.sleep();
      continue;
    }

    scan.header.stamp = ros::Time::now();
    for (int i = 0; i < data.length; ++i) {
      scan.ranges.push_back(data.ranges[i]);
    }
    laser_scan_publisher.publish(scan);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
