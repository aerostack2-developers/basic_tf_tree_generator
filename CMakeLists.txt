cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME basic_tf_tree_generator)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)


include_directories(
  include
  include/${PROJECT_NAME}
)

# set(HEADER_HPP_FILES 
#   include/${PROJECT_NAME}/rs_t265_interface.hpp  
# )

set(SOURCE_CPP_FILES 
  src/basic_tf_tree_generator.cpp
)

add_executable(${PROJECT_NAME}_node src/basic_tf_tree_generator_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp tf2 tf2_ros tf2_geometry_msgs std_msgs sensor_msgs nav_msgs geometry_msgs std_srvs as2_core as2_msgs)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
