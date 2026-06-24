cmake_minimum_required(VERSION 3.10)
project(robot_ai)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_library(RKNN_LIB rknnrt /usr/lib REQUIRED)
message(STATUS "RKNN: ${RKNN_LIB}")

add_library(rknn_engine STATIC src/rknn_engine.cpp)
target_include_directories(rknn_engine PUBLIC include ${OpenCV_INCLUDE_DIRS})

add_library(yolo_postprocess STATIC src/yolo_postprocess.cpp)
target_include_directories(yolo_postprocess PUBLIC include)

add_executable(object_detector_node src/object_detector_node.cpp)
target_link_libraries(object_detector_node rknn_engine yolo_postprocess ${RKNN_LIB}
  ${OpenCV_LIBRARIES})
target_include_directories(object_detector_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(object_detector_node rclcpp sensor_msgs vision_msgs cv_bridge)

add_executable(obstacle_publisher src/keepout_publisher.cpp)
ament_target_dependencies(obstacle_publisher rclcpp sensor_msgs vision_msgs)

install(DIRECTORY model DESTINATION share/${PROJECT_NAME})
install(TARGETS object_detector_node obstacle_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
