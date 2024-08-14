cmake_minimum_required(VERSION 3.5)

project(gtsam_bender)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find GTSAM components
find_package(GTSAM REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Build the example
add_executable(gtsam_bender_node 
  src/realsense_apriltag.cpp
)

ament_target_dependencies(gtsam_bender_node  
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  message_filters 
  tf2 
  tf2_geometry_msgs
)

target_link_libraries(gtsam_bender_node gtsam ${Boost_LIBRARIES})

install(TARGETS gtsam_bender_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
