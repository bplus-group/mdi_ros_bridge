cmake_minimum_required(VERSION 3.8)
project(mdi_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mdi_msgs REQUIRED)
find_package(mdi_lib REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include ${mdi_lib_INCLUDE_DIRS})

add_executable(mdi_rx_node
  src/mdi_publisher.cpp
  src/time_utils.cpp
)

target_link_libraries(mdi_rx_node ${rclcpp_LIBRARIES} ${mdi_lib_LIBRARIES} ${builtin_interfaces})

add_library(mdi_rx_nodelet SHARED
  ../mdi_node/src/mdi_publisher.cpp
  ../mdi_node/src/time_utils.cpp
)
target_link_libraries(mdi_rx_nodelet ${rclcpp_LIBRARIES} ${mdi_lib_LIBRARIES} ${builtin_interfaces})


target_compile_definitions(mdi_rx_nodelet PUBLIC AS_NODELET)

target_compile_definitions(mdi_rx_nodelet
  PRIVATE "COMPOSITION_BUILDING_DLL")


add_library(mdi_dummy_pub SHARED
  ../mdi_node/src/mdi_publisher.cpp
  ../mdi_node/src/time_utils.cpp
)
target_link_libraries(mdi_dummy_pub ${rclcpp_LIBRARIES} ${mdi_lib_LIBRARIES} ${builtin_interfaces})

target_compile_definitions(mdi_dummy_pub PUBLIC AS_NODELET)
target_compile_definitions(mdi_dummy_pub PUBLIC DUMMY_PUB)

target_compile_definitions(mdi_dummy_pub
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(mdi_rx_node rclcpp std_msgs mdi_msgs)
ament_target_dependencies(mdi_rx_nodelet rclcpp rclcpp_components std_msgs mdi_msgs)
ament_target_dependencies(mdi_dummy_pub rclcpp rclcpp_components std_msgs mdi_msgs)
ament_export_include_directories(include)
ament_export_dependencies(mdi_lib)
ament_export_dependencies(rosidl_default_runtime)

rclcpp_components_register_nodes(mdi_rx_nodelet "MdiReceiveNode")
rclcpp_components_register_nodes(mdi_dummy_pub "MdiReceiveNode")


install(TARGETS mdi_rx_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mdi_rx_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mdi_dummy_pub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
