cmake_minimum_required(VERSION 3.8)
project(mdi_nodelet)
message( ${PROJECT_SOURCE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mdi_msgs REQUIRED)

ADD_DEFINITIONS (
  -DAS_NODELET
)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../third_party/include
  ${CMAKE_CURRENT_LIST_DIR}/include
)
link_directories(${CMAKE_CURRENT_LIST_DIR}/../third_party/lib)

add_library(mdi_rx_nodelet SHARED ../mdi_node/src/mdi_publisher.cpp)

target_compile_definitions(mdi_rx_nodelet
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(mdi_rx_nodelet rclcpp rclcpp_components std_msgs mdi_msgs)
ament_export_include_directories(include)

rclcpp_components_register_nodes(mdi_rx_nodelet "MdiReceiveNode")

# todo: add arch branches
file(GLOB all_mdi_libs ${CMAKE_CURRENT_LIST_DIR}/../third_party/lib/lnx/x86_64/libmdirxapi.so*)
install(FILES
  ${all_mdi_libs}
  DESTINATION lib
)

install(TARGETS mdi_rx_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()
