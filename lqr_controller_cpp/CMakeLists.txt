cmake_minimum_required(VERSION 3.8)
project(lqr_controller_cpp LANGUAGES CXX)

# ── Silence CATKIN_INSTALL_INTO_PREFIX_ROOT warning ──────────────────────────
# When colcon builds a mix of catkin and ament packages it always injects
# CATKIN_INSTALL_INTO_PREFIX_ROOT.  Referencing the variable once prevents the
# “Manually-specified variables were not used by the project” warning.
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  message(STATUS "CATKIN_INSTALL_INTO_PREFIX_ROOT=${CATKIN_INSTALL_INTO_PREFIX_ROOT} (ignored)")
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ── Build ─────────────────────────────────────────────────────────────────────
add_executable(lqr_controller src/lqr_controller.cpp)
target_compile_features(lqr_controller PUBLIC cxx_std_17)
ament_target_dependencies(lqr_controller
  rclcpp
  std_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS lqr_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
