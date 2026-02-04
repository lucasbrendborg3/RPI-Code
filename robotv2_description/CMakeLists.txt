cmake_minimum_required(VERSION 3.8)
project(robotv2_description)

# Set the C++ standard to C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate and configure packages required by the project.
find_package(ament_cmake REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# Maybe add tests
ament_package()
