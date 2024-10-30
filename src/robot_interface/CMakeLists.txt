cmake_minimum_required(VERSION 3.8)
project(robot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dpd
    std_msgs
)

find_package(${dpd} REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Speed.msg"
    "msg/Speed.msg"
    DEPENDENCIES ${dpd}
)

ament_package()
