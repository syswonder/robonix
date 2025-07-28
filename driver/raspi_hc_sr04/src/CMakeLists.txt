cmake_minimum_required(VERSION 3.8)
project(piper_raspi_hc_sr04)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install configuration files
install(FILES
  sensors.yml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hc_sr04_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 