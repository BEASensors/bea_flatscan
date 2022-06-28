#include <ros/ros.h>

#include "flatscan.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bea_sensor_driver");
  ros::NodeHandle nh("~");

  bea_sensors::Flatscan flatscan(nh);

  ros::Rate rate(30.);
  while (ros::ok()) {
    flatscan.SpinOnce();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
