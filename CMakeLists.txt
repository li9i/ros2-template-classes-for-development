cmake_minimum_required(VERSION 3.8)
project(ros2_templates_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



include_directories(include
)

set(base_name template_class)
set(library_name ${base_name}_lib)
set(executable_name ${base_name}_node)
set(dependencies
  rclcpp
  std_msgs
  std_srvs)

add_library(${library_name}
  src/template_class.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
  )

add_executable(${executable_name}
  src/template_class_node.cpp)

target_link_libraries(${executable_name} ${library_name})


install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# So that `ros2 run` can find the executable
install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


################################################################################
# For custom messages and services
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  "srv/CustomSrv.srv"
  DEPENDENCIES std_msgs geometry_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

rosidl_target_interfaces(${library_name}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install message mapping rules
install(
  FILES msg/my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}/msg)

# Install service mapping rules
install(
  FILES srv/my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}/srv)
################################################################################





ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
