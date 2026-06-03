cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(odrive_ros2_control REQUIRED)
find_package(waveshare_ros2_control REQUIRED)

# Install directories
install(DIRECTORY
  config
  description
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()