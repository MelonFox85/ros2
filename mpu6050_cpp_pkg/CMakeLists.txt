cmake_minimum_required(VERSION 3.8)
project(mpu6050_cpp_pkg)

# ── find dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── build executables ───────────────────────────────────────────────
add_executable(mpu6050_node src/mpu6050_node.cpp)
ament_target_dependencies(mpu6050_node rclcpp sensor_msgs std_msgs)

add_executable(complementary_filter src/complementary_filter.cpp)
ament_target_dependencies(complementary_filter rclcpp std_msgs)

# ── install ─────────────────────────────────────────────────────────
install(TARGETS
        mpu6050_node
        complementary_filter
        DESTINATION lib/${PROJECT_NAME})

ament_package()
