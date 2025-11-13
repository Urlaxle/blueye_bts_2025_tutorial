cmake_minimum_required(VERSION 3.8)
project(bts_docking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Delcare the executable
add_executable(bts_docking_controller src/bts_docking_controller.cpp src/bts_pid.cpp)

# Link libraries
ament_target_dependencies(bts_docking_controller
  rclcpp
  nav_msgs
  std_srvs
  Eigen3
)

# Install the executable
install(TARGETS
  bts_docking_controller 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()