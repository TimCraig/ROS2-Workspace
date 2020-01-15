cmake_minimum_required(VERSION 3.5)
project(action_template_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Druai REQUIRED)
find_package(druai_idl REQUIRED)

include_directories(
../Druai/include  
)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
  "druai_idl")
  
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client
  "rclcpp"
  "rclcpp_action"
  "example_interfaces"
  "druai_idl")  

install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
