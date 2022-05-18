#include <ros/ros.h>

#include "bea_sensor_driver.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bea_sensor_driver");
  ros::NodeHandle nh;
  bea_sensors::Driver driver("/dev/ttyUSB0");  // TODO: serial port as a parameter

  ros::Rate loop_rate(10.);  // TODO: loop_rate as a parameter
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
