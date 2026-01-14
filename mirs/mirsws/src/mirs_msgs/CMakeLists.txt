cmake_minimum_required(VERSION 3.5)
project(mirs_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Params.srv"
  "srv/ParameterUpdate.srv"
  "srv/SimpleCommand.srv"
  "srv/BasicCommand.srv"
  "srv/NumberCommand.srv"
  "msg/BasicParam.msg"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
