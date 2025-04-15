cmake_minimum_required(VERSION 3.8)
project(rosbot_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ColorData.msg"
  "srv/GetGoal.srv"
  "action/MoveToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Make sure rosidl_generate_interfaces is used correctly
ament_export_dependencies(rosidl_default_runtime)

ament_package()
