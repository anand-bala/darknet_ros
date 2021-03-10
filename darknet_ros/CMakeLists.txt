cmake_minimum_required(VERSION 3.5)
project(darknet_ros)

set(CMAKE_CXX_STANDARD
    11
    CACHE STRING "The C++ standard to use"
)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find rquired packeges
find_package(OpenCV REQUIRED)
set(catkin_dependencies
    # Core dependency
    roscpp
    # Message deps
    std_msgs
    sensor_msgs
    darknet_ros_msgs
    message_generation
    # OpenCV stuff
    cv_bridge
    image_transport
    # Nodelets
    nodelet
)
find_package(catkin REQUIRED COMPONENTS ${catkin_dependencies})
add_subdirectory(libs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS ${catkin_dependencies}
)

# darknet_ros library
add_library(${PROJECT_NAME} src/darknet_ros.cc)
target_include_directories(
  ${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC DarkHelp ${catkin_LIBRARIES})
target_compile_features(
  ${PROJECT_NAME}
  PUBLIC cxx_std_11
  PRIVATE cxx_std_14
)
add_dependencies(${PROJECT_NAME} darknet_ros_msgs_generate_messages_cpp)

# Object detector node
add_executable(object_detector src/object_detector_node.cpp)
target_link_libraries(object_detector PRIVATE ${PROJECT_NAME})

# Darknet ROS Nodelets
add_library(${PROJECT_NAME}_nodelet src/object_detector_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet PRIVATE ${PROJECT_NAME})
target_compile_features(
  ${PROJECT_NAME}_nodelet
  PUBLIC cxx_std_11
  PRIVATE cxx_std_14
)

install(
  TARGETS ${PROJECT_NAME} object_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/darknet_ros
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch yolo_network_config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Download yolov2-tiny.weights
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
set(FILE "${PATH}/yolov2-tiny.weights")
message(STATUS "Checking and downloading yolov2-tiny.weights if needed ...")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND
      wget -q
      https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov2-tiny.weights
      -P ${PATH}
  )
endif()

# Download yolov3.weights
set(FILE "${PATH}/yolov3.weights")
message(STATUS "Checking and downloading yolov3.weights if needed ...")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND
      wget -q
      https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov3.weights
      -P ${PATH}
  )
endif()
