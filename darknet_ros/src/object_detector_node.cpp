#include "darknet_ros/darknet_ros.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");

  darknet_ros::DarknetNode detector(nodeHandle);

  ros::spin();
  return 0;
}
