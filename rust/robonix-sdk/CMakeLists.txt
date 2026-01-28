cmake_minimum_required(VERSION 3.20)
# Silence CMP0148 warning from rosidl_generator_py / FindPython
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

project(robonix_sdk)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces from .msg and .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
# messages
  "msg/RelationType.msg"
  "msg/Relation.msg"
  "msg/TextureType.msg"
  "msg/BoundingBox.msg"
  "msg/Point3D.msg"
  "msg/FrameMapping.msg"
  "msg/Object.msg"
  "msg/SpatialMapEntry.msg"
  "msg/Dict.msg"
  "msg/Occupancy3D.msg"
  "msg/PrimitiveInstance.msg"
  "msg/ServiceInstance.msg"
  "msg/SkillInstance.msg"
# services - robonix API
  # Primitive services
  "srv/primitive/RegisterPrimitive.srv"
  "srv/primitive/QueryPrimitive.srv"
  # Service services
  "srv/service/RegisterService.srv"
  "srv/service/QueryService.srv"
  # Standard service definitions
  "srv/service/spatial_map/GetSpatialMap.srv"
  "srv/service/semantic_map/QuerySemanticMap.srv"
  "srv/service/task_plan/PlanTask.srv"
  "srv/service/plan_simulate/SimulatePlan.srv"
  "srv/service/result_feedback/ResultFeedback.srv"
  # Skill services
  "srv/skill/RegisterSkill.srv"
  "srv/skill/QuerySkill.srv"
  # Task services
  "srv/task/SubmitTask.srv"
  "srv/task/TaskData.srv"
  # Test services
  "srv/PingPong.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    builtin_interfaces
)


# Install Python client helper (RobonixClient) into same dir as rosidl-generated
# package, so "source install/setup.bash" gives robonix_sdk.client (no global install).
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Match ament/rosidl layout: local/lib/pythonX.Y/dist-packages/robonix_sdk
set(_rbnx_py_pkg "local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages/robonix_sdk")
install(FILES python/client.py DESTINATION ${_rbnx_py_pkg})

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

