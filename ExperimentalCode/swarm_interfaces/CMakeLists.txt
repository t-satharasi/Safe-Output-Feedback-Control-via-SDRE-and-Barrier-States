cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# CRITICAL: Generate interfaces FIRST, before any executables
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConnectAgent.srv"
  "srv/DisconnectAgent.srv"
  "srv/StartExperiment.srv"
  "msg/ConnectedAgents.msg"
  "msg/Heartbeat.msg"
  "msg/PrepareExperimentCommand.msg"
  "msg/PrepareExperimentResponse.msg"
  "msg/InitiateTakeoffCommand.msg"
  "msg/InitiateTakeoffResponse.msg"
  "msg/StartExperimentCommand.msg"
  "msg/InitiateLandCommand.msg"
  "msg/InitiateLandResponse.msg"
)

# Set the C++ standard for this package
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
