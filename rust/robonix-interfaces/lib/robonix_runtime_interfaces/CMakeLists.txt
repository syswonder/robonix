cmake_minimum_required(VERSION 3.8)
project(robonix_runtime_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModuleDescriptor.msg"
  "msg/InterfaceDescriptor.msg"
  "msg/InterfaceInstance.msg"
  "msg/RegisterModuleRequest.msg"
  "msg/RegisterModuleResponse.msg"
  "msg/RegisterInterfaceRequest.msg"
  "msg/RegisterInterfaceResponse.msg"
  "msg/ResolveInterfaceRequest.msg"
  "msg/ResolveInterfaceResponse.msg"
  "msg/ListInterfacesRequest.msg"
  "msg/ListInterfacesResponse.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
