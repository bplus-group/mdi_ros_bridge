cmake_minimum_required(VERSION 3.8)
project(mdi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvetoTimebase.msg"
  "msg/AvetoFrame.msg"
  "msg/MdiAvetoProfile.msg"
  "msg/Mdirawframe.msg"
  "msg/MdiCsi2Frame.msg"
  "msg/MdiI2cFrame.msg"
  "msg/MdiStatusFrame.msg"

  "msg/Mdirxapistatus.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_package()