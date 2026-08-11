cmake_minimum_required(VERSION 3.5)
project(autonomy_park_viz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/autonomy_park_viz_node.cpp
    src/AutonomyParkViz.cpp
    include/AutonomyParkViz.hpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

