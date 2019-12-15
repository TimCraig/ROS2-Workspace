cmake_minimum_required(VERSION 3.5)
project(druai_idl_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Druai REQUIRED)
find_package(druai_idl REQUIRED)

include_directories(
../Druai/include  
)

add_executable(subscriber src/subscriber.cpp)
target_include_directories(subscriber
  PUBLIC
  ${druai_idl_INCLUDE_DIRS}
  include
)
ament_target_dependencies(subscriber
   rclcpp
   std_msgs
   Druai
   druai_idl
)

add_executable(publisher src/publisher.cpp)
target_include_directories(publisher
  PUBLIC
  ${druai_idl_INCLUDE_DIRS}
  include
)
ament_target_dependencies(publisher
   rclcpp
   std_msgs
   Druai
#   DruaiMsg
   druai_idl
)

add_executable(druai_server src/druai_server.cpp)
ament_target_dependencies(druai_server rclcpp druai_idl)

add_executable(druai_client src/druai_client.cpp)
ament_target_dependencies(druai_client rclcpp druai_idl)

add_executable(druai_action_server src/druai_action_server.cpp)
ament_target_dependencies(druai_action_server rclcpp rclcpp_action druai_idl)

add_executable(druai_action_client src/druai_action_client.cpp)
ament_target_dependencies(druai_action_client rclcpp rclcpp_action druai_idl)


install(TARGETS
  subscriber
  publisher
  druai_server
  druai_client
  druai_action_server
  druai_action_client
  DESTINATION lib/${PROJECT_NAME}
)
  

ament_package()
