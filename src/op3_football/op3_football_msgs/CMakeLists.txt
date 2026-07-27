cmake_minimum_required(VERSION 3.8)
project(op3_football_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTick.msg"
  "msg/JointTickArray.msg"
  "srv/JointWrite.srv"
  "srv/JointRead.srv"
  "srv/JointWriteMany.srv"
  "srv/SetModule.srv"
  "srv/WalkingCommand.srv"
  "srv/SetWalkingParams.srv"
  "srv/EmptyTrigger.srv"
  "srv/SetLed.srv"
  "srv/SetTorque.srv"
  DEPENDENCIES std_msgs op3_walking_module_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
