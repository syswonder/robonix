cmake_minimum_required(VERSION 3.20)
project(robonix_core)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces from .msg and .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
# messages
  "msg/RelationType.msg"
  "msg/Relation.msg"
  "msg/TextureType.msg"
  "msg/BoundingBox.msg"
  "msg/Point3D.msg"
  "msg/FrameMapping.msg"
  "msg/Entity.msg"
  "msg/SpatialMapEntry.msg"
# services
  "srv/RegisterCapSkl.srv"
  "srv/QueryCapSkl.srv"
  "srv/RegisterModel.srv"
  "srv/QueryModel.srv"
  "srv/CreateTask.srv"
  "srv/GetTask.srv"
  "srv/ListTasks.srv"
  "srv/CancelTask.srv"
  "srv/GetSemanticMap.srv"
  "srv/AddEntity.srv"
  "srv/GetSpatialMap.srv"
  "srv/AddSpatialMapEntry.srv"
  "srv/GetMapStatus.srv"
  DEPENDENCIES
)

# Install Python client library
# Install to Python site-packages (local/lib/python3.10/dist-packages)
# This ensures the package can be imported via PYTHONPATH
install(DIRECTORY robonix_core
  DESTINATION local/lib/python3.10/dist-packages
  FILES_MATCHING
  PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

