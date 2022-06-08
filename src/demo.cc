#include <ros/ros.h>

#include "flat_scan.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bea_sensor_driver");
  ros::NodeHandle nh("~");

  bea_sensors::FlatScan flat_scan(nh);

  // sensor_msgs::LaserScan scan;
  // scan.header.frame_id = frame_id;
  // scan.angle_min = angles::from_degrees(first_angle);
  // scan.angle_max = angles::from_degrees(last_angle);
  // scan.angle_increment = angles::from_degrees(resolution);
  // scan.range_min = min_range;
  // scan.range_max = max_range;
  // scan.scan_time = refresh_period;
  // scan.time_increment = refresh_period / number_of_spots;

  float loop_rate;
  nh.param("loop_rate", loop_rate, static_cast<float>(10.));
  ros::Rate rate(loop_rate);
  while (ros::ok()) {
    // const bea_sensors::LaserData& data{flat_scan.laser_data()};
    // if (data.length == 0) {
    //   ros::spinOnce();
    //   rate.sleep();
    //   continue;
    // }

    // scan.header.stamp = ros::Time::now();
    // scan.ranges.clear();
    // for (int i = 0; i < data.length; ++i) {
    //   scan.ranges.push_back(data.ranges[i]);
    // }
    // scan.intensities.clear();
    // for (int i = 0; i < data.length; ++i) {
    //   scan.intensities.push_back(data.intensities[i]);
    // }
    // laser_scan_publisher.publish(scan);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
