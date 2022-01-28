cmake_minimum_required(VERSION 3.8)
project(mdi_node)
message( ${PROJECT_SOURCE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mdi_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../third_party/include)
link_directories(${CMAKE_CURRENT_LIST_DIR}/../third_party/lib)

add_executable(mdi_rx_node 
  src/mdi_publisher.cpp
  src/mdi_lib_wrapper.cpp
)
ament_target_dependencies(mdi_rx_node rclcpp std_msgs mdi_msgs)
ament_export_include_directories(include)

# todo: add arch branches
file(GLOB all_mdi_libs ${CMAKE_CURRENT_LIST_DIR}/../third_party/lib/lnx/x86_64/libmdirxapi.so*)
install(FILES
  ${all_mdi_libs}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mdi_rx_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# this is the magic to actually include your message as a header!
#rosidl_target_interfaces(mdi_rx_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()