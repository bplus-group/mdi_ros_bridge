cmake_minimum_required(VERSION 3.8)
project(mdi_daq_converter)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mdi_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../third_party/include
  ${CMAKE_CURRENT_LIST_DIR}/include
)

add_library(${PROJECT_NAME} SHARED 
  src/mdi_image_converter.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components std_msgs sensor_msgs mdi_msgs)
#ament_export_include_directories(include)

rclcpp_components_register_nodes(${PROJECT_NAME} "MdiConverterNode")

install(TARGETS ${PROJECT_NAME}
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
