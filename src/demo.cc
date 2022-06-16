#include <ros/ros.h>

#include "flat_scan.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bea_sensor_driver");
  ros::NodeHandle nh("~");

  bea_sensors::FlatScan flat_scan(nh);

  ros::Rate rate(30.);
  while (ros::ok()) {
    flat_scan.SpinOnce();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
