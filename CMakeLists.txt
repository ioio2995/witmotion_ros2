cmake_minimum_required(VERSION 3.8)
project(witmotion_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
)

set(SOURCES
  src/witmotion_node.cpp
  src/witmotion_handler.cpp
)

add_executable(witmotion_ros2 ${SOURCES})

ament_target_dependencies(witmotion_ros2
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
  tf2
)

target_link_libraries(witmotion_ros2
  ${Boost_LIBRARIES}
)

install(TARGETS witmotion_ros2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})  

ament_package()

