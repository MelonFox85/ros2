cmake_minimum_required(VERSION 3.8)
project(feedback_cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(neg_feedback src/neg_feedback.cpp)
ament_target_dependencies(neg_feedback rclcpp std_msgs)

install(TARGETS neg_feedback
  DESTINATION lib/${PROJECT_NAME})

ament_package()
