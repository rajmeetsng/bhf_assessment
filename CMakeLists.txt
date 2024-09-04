cmake_minimum_required(VERSION 3.5)
project(bhf_vo_new)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#rosidl_generate_interfaces(${PROJECT_NAME}
   #"msg/VisualOdometry.msg"
#)

add_executable(bhf_vo_new src/camera_pose_publisher.cpp)
#add_executable(bhf_vo_new src/float_publisher.cpp)



ament_target_dependencies(bhf_vo_new rclcpp rmw sensor_msgs std_msgs image_transport cv_bridge OpenCV Eigen3)

install(TARGETS
  bhf_vo_new
  DESTINATION lib/${PROJECT_NAME})

#ament_export_dependencies(rosidl_default_runtime)

ament_package()
