cmake_minimum_required(VERSION 3.5)
project(druai_idl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)

#Needed to handle custom messages
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#set(FastRTPS_INCLUDE_DIR:PATH /opt/ros/dashing/include)

#Generate the include file and other support for the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DruaiMsg.msg"
  "srv/DruaiSrv.srv"
  "action/DruaiAction.action"
  DEPENDENCIES builtin_interfaces) 

ament_export_dependencies(rosidl_default_runtime)
ament_package()
