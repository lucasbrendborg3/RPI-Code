cmake_minimum_required(VERSION 3.16)
project(waveshare_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS 
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
include_directories(../waveshare_base/include)

ament_auto_add_library(
  waveshare_ros2_control_plugin SHARED
  ../waveshare_base/src/SCSCL.cpp
  ../waveshare_base/src/SCS.cpp
  ../waveshare_base/src/SCSerial.cpp
  ../waveshare_base/src/SMSBL.cpp
  ../waveshare_base/src/SMSCL.cpp
  ../waveshare_base/src/SMS_STS.cpp
  src/waveshare_hardware_interface.cpp
)

ament_target_dependencies(waveshare_ros2_control_plugin
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface waveshare_hardware_interface.xml)

target_compile_features(waveshare_ros2_control_plugin PRIVATE cxx_std_17)

install(TARGETS waveshare_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()