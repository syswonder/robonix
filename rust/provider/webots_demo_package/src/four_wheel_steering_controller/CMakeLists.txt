cmake_minimum_required(VERSION 3.5)
project(four_wheel_steering_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(four_wheel_steering_controller SHARED
  src/four_wheel_steering_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

pluginlib_export_plugin_description_file(controller_interface four_wheel_steering_controller_plugin.xml)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(ament_lint_auto REQUIRED)

#   ament_lint_auto_find_test_dependencies()

#   ament_add_gmock(test_four_wheel_steering_controller
#     test/test_four_wheel_steering_controller.cpp
#     ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_four_wheel_steering_controller.yaml)
#   target_link_libraries(test_four_wheel_steering_controller
#     four_wheel_steering_controller
#   )
#   ament_target_dependencies(test_four_wheel_steering_controller
#     ${four_wheel_steering_controller_FOUND_TEST_DEPENDS}
#   )

#   ament_add_gmock(test_four_wheel_steering_model
#     test/test_four_wheel_steering_model.cpp)
#   target_link_libraries(test_four_wheel_steering_model
#     four_wheel_steering_controller
#   )
#   ament_target_dependencies(test_four_wheel_steering_model
#     ${four_wheel_steering_controller_FOUND_TEST_DEPENDS}
#   )
# endif()

ament_auto_package()

# From ROS 2 Kilted Kaiju onwards ament_auto_package will install headers into
# include/${PROJECT_NAME} by default. For older distributions we enable the
# same behavior by setting USE_SCOPED_HEADER_INSTALL_DIR to ON so headers are
# installed into include/four_wheel_steering_controller instead of plain
# include/. This avoids collisions when multiple packages install headers.
# See the ament_cmake_auto documentation for details.
set(USE_SCOPED_HEADER_INSTALL_DIR ON)
