cmake_minimum_required(VERSION 3.8) # Changed from 3.16 for broader compatibility, 3.8 is common for ROS 2
project(waveshare_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # For the executables

# --- waveshare_servo_lib (SCS, SMS communication) ---
add_library(waveshare_servo_lib SHARED # Or STATIC if you prefer and it's only used by executables in this package
  src/SCSCL.cpp
  src/SCS.cpp
  src/SCSerial.cpp
  src/SMSBL.cpp
  src/SMSCL.cpp
  src/SMS_STS.cpp
)
target_compile_features(waveshare_servo_lib PUBLIC cxx_std_17) # Assuming C++17
target_include_directories(waveshare_servo_lib PUBLIC
  # For compiling the library: makes files in waveshare_base/include/ available
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # For other packages using this library: headers will be in install/waveshare_base/include/waveshare_base/
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# If waveshare_servo_lib itself had ROS dependencies:
# ament_target_dependencies(waveshare_servo_lib PUBLIC rclcpp) # Example

# --- calibrate_midpoint executable ---
add_executable(calibrate_midpoint src/calibrate_midpoint.cpp)
target_include_directories(calibrate_midpoint PUBLIC
  # For compiling the executable: makes files in waveshare_base/include/ available
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(calibrate_midpoint rclcpp)
target_link_libraries(calibrate_midpoint waveshare_servo_lib)

# --- set_id executable ---
add_executable(set_id src/set_id.cpp)
target_include_directories(set_id PUBLIC
  # For compiling the executable: makes files in waveshare_base/include/ available
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(set_id rclcpp)
target_link_libraries(set_id waveshare_servo_lib)

# --- Installation ---

# Install headers from the 'include' directory
# They will be installed to install/waveshare_base/include/waveshare_base/
# So, other packages would use #include "waveshare_base/SCServo.h"
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" # Only install header files
)

# Install the library
install(TARGETS waveshare_servo_lib
  EXPORT export_waveshare_servo_lib # For ament_export_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # Typically for DLLs on Windows if SHARED
)

# Install the executables
install(
  TARGETS calibrate_midpoint set_id
  DESTINATION lib/${PROJECT_NAME} # Standard location for ROS executables
)

# --- Exports (for other ROS packages to use this package) ---
# Export the library target so other packages can link against waveshare_servo_lib
ament_export_targets(export_waveshare_servo_lib HAS_LIBRARY_TARGET)

# Export include directories for packages depending on this one.
# This tells downstream packages to look in install/waveshare_base/include/waveshare_base
ament_export_include_directories(include/${PROJECT_NAME})

# Export dependencies that downstream packages might also need if they use this package's library/headers
ament_export_dependencies(rclcpp) # If the library's public headers expose rclcpp types, or if it's a common dep

ament_package()