cmake_minimum_required(VERSION 3.5)
project(robonix_stress_test_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robonix_sdk REQUIRED)

add_executable(stress_test stress_test.cpp)
ament_target_dependencies(stress_test
  rclcpp
  robonix_sdk
)

install(TARGETS
  stress_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
