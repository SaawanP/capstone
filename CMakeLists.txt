cmake_minimum_required(VERSION 3.5)
project(capstone)

# Find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  capstone/brain.py
  capstone/camera.py
  capstone/defect_ML.py
  capstone/motor_controller.py
  capstone/video_feed.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()