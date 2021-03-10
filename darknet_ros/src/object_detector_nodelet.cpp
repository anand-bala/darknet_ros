#include "darknet_ros/darknet_ros.hpp"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <memory>

namespace darknet_ros {

class ObjectDetectorNodelet : public nodelet::Nodelet {
  std::unique_ptr<DarknetNode> m_darknet;

 public:
  ObjectDetectorNodelet() = default;

 private:
  void onInit() override {
    ros::NodeHandle nodeHandle("~");
    m_darknet = std::make_unique<DarknetNode>(nodeHandle);
  }
};
} // namespace darknet_ros

// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(darknet_ros::ObjectDetectorNodelet, nodelet::Nodelet);
