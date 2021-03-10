#include "darknet_ros/darknet_ros.hpp"

#include <array>
#include <memory>
#include <random>

namespace darknet_ros {

constexpr auto DEFAULT_CAMERA_TOPIC         = "/camera/image_raw";
constexpr auto DEFAULT_IMAGE_SUB_QUEUE_SIZE = 1;
constexpr auto DEFAULT_IMAGE_PUB_QUEUE_SIZE = 1;
constexpr auto DEFAULT_BBOX_PUB_QUEUE_SIZE  = 1;

DarknetNode::DarknetNode(const ros::NodeHandle& nh) : m_nh{nh}, m_image_transport{nh} {
  // Initialize the pub-sub stuff for the images
  m_image_sub = m_image_transport.subscribe(
      DEFAULT_CAMERA_TOPIC,
      DEFAULT_IMAGE_SUB_QUEUE_SIZE,
      &DarknetNode::handle_image,
      this);
  m_image_pub =
      m_image_transport.advertise("detection_image", DEFAULT_IMAGE_PUB_QUEUE_SIZE);
  // Initialize bbox publisher
  m_bbox_pub = m_nh.advertise<darknet_ros_msgs::BoundingBoxes>(
      "bounding_boxes", DEFAULT_BBOX_PUB_QUEUE_SIZE);

  // ID for the GPU to use
  int gpu_id = 0;
  m_nh.param("yolo_model/gpu_id", gpu_id, 0);
  // Threshold of object detection.
  m_nh.param("yolo_model/threshold", m_threshold, DEFAULT_DETECTION_THRESHOLD);

  // Path to config file.
  std::string config_file;
  if (m_nh.getParam("yolo_model/config_file", config_file)) {
    ROS_INFO("Using config from: %s", config_file.c_str());
  } else {
    ROS_ERROR("Undefined config file for YOLO");
  }
  // Path to weights file.
  std::string weights_file;
  if (m_nh.getParam("yolo_model/weights_file", weights_file)) {
    ROS_INFO("Using weights from: %s", weights_file.c_str());
  } else {
    ROS_ERROR("Undefined weights file for YOLO");
  }

  // Initialize Darknet detector
  m_detector = std::make_unique<Detector>(config_file, weights_file, gpu_id);

  // Get the classes present in the detection.
  m_nh.param(
      "yolo_model/detection_classes", m_class_labels, std::vector<std::string>{});
}

void DarknetNode::handle_image(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Get detections from darknet
  auto detections = m_detector->detect(cv_ptr->image, m_threshold);
  // Convert the detections to messages.
  auto bboxes = get_boxes(detections);
  // Draw the bounding boxes
  draw_boxes(cv_ptr, detections);
  // Output modified video stream
  m_image_pub.publish(cv_ptr->toImageMsg());
  // Publish the bounding boxes
  m_bbox_pub.publish(bboxes);
}

void DarknetNode::draw_boxes(
    cv_bridge::CvImagePtr& cv_ptr,
    const std::vector<bbox_t>& bounding_boxes) {
  constexpr double max_hex     = 255;
  constexpr int label_offset   = 10;
  constexpr float font_scale   = 0.9;
  constexpr int font_thickness = 2;

  auto rd   = std::default_random_engine{};
  auto dist = std::uniform_real_distribution<double>{0, max_hex};
  // For each box,
  for (const auto& box : bounding_boxes) {
    auto color        = cv::Scalar{dist(rd), dist(rd), dist(rd)};
    std::string name  = m_class_labels.at(box.obj_id);
    std::string label = name + ": " + std::to_string(box.prob);
    // Draw a rectangle.
    auto x = static_cast<int>(box.x);
    auto y = static_cast<int>(box.y);
    auto w = static_cast<int>(box.w);
    auto h = static_cast<int>(box.h);
    cv::rectangle(cv_ptr->image, cv::Point{x, y}, cv::Point{x + w, y + h}, color);
    cv::putText(
        cv_ptr->image,
        label,
        cv::Point{x, y - label_offset},
        cv::FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        font_thickness);
  }
}

std::vector<darknet_ros_msgs::BoundingBox>
DarknetNode::get_boxes(const std::vector<bbox_t>& bounding_boxes) {
  std::vector<darknet_ros_msgs::BoundingBox> ret;
  for (const auto& box : bounding_boxes) {
    darknet_ros_msgs::BoundingBox msg;
    msg.id          = box.obj_id;
    msg.Class       = m_class_labels.at(box.obj_id);
    msg.xmin        = box.x;
    msg.ymin        = box.y;
    msg.xmax        = box.x + box.w;
    msg.ymax        = box.y + box.h;
    msg.probability = box.prob;

    ret.push_back(msg);
  }
  return ret;
}

} // namespace darknet_ros
