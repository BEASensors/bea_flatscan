#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "bea_sensor_driver.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bea_sensor_driver");
  ros::NodeHandle nh;
  bea_sensors::Driver driver("/dev/ttyUSB0");  // TODO: serial port as a parameter

  ros::Publisher laser_scan_publisher = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);

  ros::Rate loop_rate(10.);  // TODO: loop_rate as a parameter
  while (ros::ok()) {
    const bea_sensors::Laser laser{driver.laser()};
    if (laser.length != 0) {
      sensor_msgs::LaserScan scan;
      scan.header.stamp = ros::Time::now();
      scan.header.frame_id = "laser";
      scan.angle_min = laser.angle_min;
      scan.angle_max = laser.angle_max;
      scan.angle_increment = laser.angle_increment;
      scan.range_min = 0.;
      scan.range_max = 8.;
      // scan.scan_time = 1. / 23.25;
      // scan.time_increment = 1. / 23.25 / 400;
      for (int i = 0; i < laser.length; ++i) {
        scan.ranges.push_back(laser.ranges[i]);
      }

      laser_scan_publisher.publish(scan);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
