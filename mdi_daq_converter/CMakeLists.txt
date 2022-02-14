cmake_minimum_required(VERSION 3.8)
project(mdi_daq_converter)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
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

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components camera_calibration_parsers std_msgs sensor_msgs image_transport mdi_msgs)
#ament_export_include_directories(include)

rclcpp_components_register_nodes(${PROJECT_NAME} "MdiConverterNode")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}_unit_tests test/unittests.cpp)
  target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME})
endif()


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
