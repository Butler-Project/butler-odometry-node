cmake_minimum_required(VERSION 3.8)
project(odometry_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++23 to match workspace convention
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(imu_driver_node REQUIRED)
find_package(ddsm115_driver REQUIRED)

set(DEPS
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  imu_driver_node
  ddsm115_driver
)

# Component library
add_library(odometry_node_component SHARED src/odometry_node.cpp)
target_compile_definitions(odometry_node_component PRIVATE COMPONENT_ONLY)
target_include_directories(odometry_node_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(odometry_node_component ${DEPS})
rclcpp_components_register_nodes(odometry_node_component "OdometryNode")

# Standalone executable
add_executable(odometry_node src/odometry_node.cpp)
target_include_directories(odometry_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(odometry_node ${DEPS})

install(TARGETS odometry_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
