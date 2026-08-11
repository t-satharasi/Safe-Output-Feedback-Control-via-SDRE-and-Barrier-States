cmake_minimum_required(VERSION 3.8)
project(px4_safety_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(
    ${PROJECT_NAME}
    src/PX4Safety.cpp
    include/px4_safety_lib/PX4Safety.hpp
)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    geometry_msgs
    visualization_msgs
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "px4_safety_lib_BUILDING_LIBRARY")

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(
    include
)

ament_export_libraries(
    ${PROJECT_NAME}
)

ament_export_targets(
    export_${PROJECT_NAME}
)

install(
    DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
