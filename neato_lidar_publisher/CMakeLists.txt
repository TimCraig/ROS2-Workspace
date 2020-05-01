cmake_minimum_required(VERSION 3.5)
project(neato_lidar_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Druai REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
../Druai/include  
)

add_executable(neato_lidar_publisher src/NeatoLidarPublisher.cpp src/GetSurrealComm.cpp)
ament_target_dependencies(neato_lidar_publisher rclcpp sensor_msgs Boost)

add_executable(neato_lidar_image src/NeatoLidarImage.cpp src/LidarScanImage.cpp)
ament_target_dependencies(neato_lidar_image rclcpp sensor_msgs Boost OpenCV) 

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

ament_export_dependencies(Boost)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(TARGETS
  neato_lidar_publisher
  neato_lidar_image
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
