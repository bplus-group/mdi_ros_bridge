cmake_minimum_required(VERSION 3.8)
project(mdi_lib)
message( ${PROJECT_SOURCE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
)

add_library(mdirxapi SHARED IMPORTED)
set_property(TARGET mdirxapi PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/${CMAKE_SYSTEM_NAME}/libmdirxapi.so")


ament_export_include_directories(include)
ament_export_libraries(mdirxapi)
#ament_export_targets(mdirxapiTargets HAS_LIBRARY_TARGET)


file(GLOB all_mdi_libs "${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/${CMAKE_SYSTEM_NAME}/libmdirxapi.so*")
install(FILES
  ${all_mdi_libs}
  DESTINATION lib
)

install(DIRECTORY include/
	  DESTINATION include)
  #install(
  #  EXPORT mdirxapiTargets DESTINATION lib
  #)


ament_package()
