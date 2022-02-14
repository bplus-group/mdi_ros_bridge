cmake_minimum_required(VERSION 3.8)
project(mdi_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mdi_msgs REQUIRED)
find_package(mdi_lib REQUIRED)

include_directories(include ${mdi_lib_INCLUDE_DIRS})

add_executable(mdi_rx_node 
  src/mdi_publisher.cpp
  src/time_utils.cpp
)

target_link_libraries(mdi_rx_node ${rclcpp_LIBRARIES} ${mdi_lib_LIBRARIES} ${builtin_interfaces})

ament_target_dependencies(mdi_rx_node rclcpp std_msgs mdi_msgs)
ament_export_include_directories(include)
ament_export_dependencies(mdi_lib)


install(TARGETS mdi_rx_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



ament_package()
