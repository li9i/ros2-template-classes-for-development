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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
