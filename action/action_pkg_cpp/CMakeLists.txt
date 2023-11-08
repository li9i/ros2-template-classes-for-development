cmake_minimum_required(VERSION 3.8)
project(action_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_pkg_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# ------------------------------------------------------------------------------
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)

target_include_directories(action_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(action_server
  "action_pkg_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_pkg_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)

# ------------------------------------------------------------------------------
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)

target_include_directories(action_client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(action_client
  "action_pkg_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_pkg_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)


install(TARGETS
  action_server
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
