/// @file  darknet_ros/darknet_ros.hpp
/// @brief Define an API that exposes ROS node-like capabilities in a unified interface.

#pragma once
#ifndef DARKNET_ROS_DARKNET_ROS_HPP
#define DARKNET_ROS_DARKNET_ROS_HPP

#include <memory>
#include <string>
#include <vector>

// ROS stuff
#include <ros/ros.h>

// Yolo stuff
#include <yolo_v2_class.hpp>

// Darknet messages
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// Image transporter, messages, and OpenCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

namespace darknet_ros {
/// Default detection threshold for YOLO
constexpr float DEFAULT_DETECTION_THRESHOLD = 0.3;

/// High-level object that deals with pub-sub of darknet related topics.
class DarknetNode {
 private:
  /// ROS node handle
  ros::NodeHandle m_nh;
  /// Image transporter
  image_transport::ImageTransport m_image_transport;
  /// Subscriber to an image stream
  image_transport::Subscriber m_image_sub;
  /// Publisher of an image stream with annotated detections
  image_transport::Publisher m_image_pub;
  /// Bounding boxes publisher
  ros::Publisher m_bbox_pub;

  /// YOLO Darknet detection threshold
  float m_threshold = DEFAULT_DETECTION_THRESHOLD;
  /// YOLO Darknet detector
  std::unique_ptr<Detector> m_detector;
  /// The labels/classes of objects.
  std::vector<std::string> m_class_labels;

  /// Callback for image stream subscriber
  void handle_image(const sensor_msgs::ImageConstPtr& msg);

  /// Helper to draw detected bounding boxes on the output image
  void draw_boxes(cv_bridge::CvImagePtr&, const std::vector<bbox_t>&);

  /// Helper to draw detected bounding boxes on the output image
  std::vector<darknet_ros_msgs::BoundingBox> get_boxes(const std::vector<bbox_t>&);

 public:
  DarknetNode(const ros::NodeHandle& nh);
};

} // namespace darknet_ros

#endif /* end of include guard: DARKNET_ROS_DARKNET_ROS_HPP */
