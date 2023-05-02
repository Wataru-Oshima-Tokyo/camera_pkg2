cmake_minimum_required(VERSION 3.8)
project(camera_pkg2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies

#find_package(OpenCV 4 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find dependencies
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(opencv2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Coordinate.msg"
)


add_executable(ros_img src/ros_image.cpp)
ament_target_dependencies(ros_img rclcpp std_msgs OpenCV sensor_msgs image_transport cv_bridge)


install(TARGETS
  ros_img
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    msg
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
