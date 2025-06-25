cmake_minimum_required(VERSION 3.8)
project(motor_controller_cpp LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# system libraries
find_library(PIGPIO_LIB      pigpio         REQUIRED)
find_library(PIGPIOD_IF2_LIB pigpiod_if2    REQUIRED)

add_executable(motor_controller src/motor_controller.cpp)
target_compile_features(motor_controller PUBLIC cxx_std_17)

target_link_libraries(motor_controller
  ${PIGPIO_LIB}
  ${PIGPIOD_IF2_LIB}
)

ament_target_dependencies(motor_controller rclcpp std_msgs)

install(TARGETS motor_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
