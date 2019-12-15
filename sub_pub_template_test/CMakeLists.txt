cmake_minimum_required(VERSION 3.5)
project(sub_pub_template_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Druai REQUIRED)

include_directories(
../Druai/include  
)

add_executable(subscriber_template src/subscriber.cpp)
ament_target_dependencies(subscriber_template rclcpp std_msgs Druai)

add_executable(publisher_template src/publisher.cpp)
ament_target_dependencies(publisher_template rclcpp std_msgs Druai)

install(TARGETS
  subscriber_template
  publisher_template
  DESTINATION lib/${PROJECT_NAME})
  
  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
