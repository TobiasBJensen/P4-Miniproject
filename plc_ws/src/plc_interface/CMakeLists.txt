cmake_minimum_required(VERSION 3.8)
project(plc_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Carrier.msg"
  "msg/EstTime.msg"
  "msg/Time.msg"
  "msg/NodeCheck.msg"
  "msg/NodeBool.msg"
  "srv/SetLed.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
