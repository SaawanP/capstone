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
    geometry_msgs
    sensor_msgs
)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RobotSpeed.msg"
    "msg/CameraSpeed.msg"
    "msg/Defect.msg"
    "msg/Save.msg"
    DEPENDENCIES ${dpd}
)

ament_package()
