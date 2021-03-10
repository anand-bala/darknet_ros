#include "darknet_ros/darknet_ros.hpp"

#include <array>
#include <memory>
#include <random>

namespace darknet_ros {

constexpr auto DEFAULT_CAMERA_TOPIC         = "/camera/image_raw";
constexpr auto DEFAULT_IMAGE_SUB_QUEUE_SIZE = 1;
constexpr auto DEFAULT_IMAGE_PUB_QUEUE_SIZE = 1;
constexpr auto DEFAULT_BBOX_PUB_QUEUE_SIZE  = 1;

static std::vector<darknet_ros_msgs::BoundingBox>
get_boxes(const DarkHelp::PredictionResults& bounding_boxes) {
  std::vector<darknet_ros_msgs::BoundingBox> ret;
  for (const auto& box : bounding_boxes) {
    darknet_ros_msgs::BoundingBox msg;
    msg.probability = box.best_probability;
    msg.id          = box.best_class;
    msg.Class       = box.name;
    msg.xmin        = box.rect.x;
    msg.ymin        = box.rect.y;
    msg.xmax        = box.rect.x + box.rect.width;
    msg.ymax        = box.rect.y + box.rect.height;

    ret.push_back(msg);
  }
  return ret;
}

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

  // Path to names file
  std::string names_file = "";
  m_nh.param("yolo_model/names_file", names_file, std::string{""});
  ROS_INFO("Using names from: %s", names_file.c_str());

  // Threshold of object detection.
  float threshold = DEFAULT_DETECTION_THRESHOLD;
  m_nh.param("yolo_model/threshold", threshold, DEFAULT_DETECTION_THRESHOLD);

  // Initialize Darknet detector
  m_detector = std::make_unique<DarkHelp>(config_file, weights_file, names_file);
  m_detector->threshold = threshold;
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
  auto detections = m_detector->predict(cv_ptr->image);
  // Convert the detections to messages.
  auto bboxes = get_boxes(detections);
  // Draw the bounding boxes
  cv_ptr->image = m_detector->annotate();
  // Output modified video stream
  m_image_pub.publish(cv_ptr->toImageMsg());
  // Publish the bounding boxes
  m_bbox_pub.publish(bboxes);
}

} // namespace darknet_ros
