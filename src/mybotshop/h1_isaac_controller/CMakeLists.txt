cmake_minimum_required(VERSION 3.5)
project(h1_isaac_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(AMENT_DEPENDENCIES rclpy tf2 tf2_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS scripts/isaac_node
                 DESTINATION lib/${PROJECT_NAME}
)

ament_package()
