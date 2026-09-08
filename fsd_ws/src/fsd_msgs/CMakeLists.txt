cmake_minimum_required(VERSION 3.8)
project(fsd_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConeDetection2D.msg"
  "msg/ConeDetection2DArray.msg"
  "msg/Cone3D.msg"
  "msg/Cone3DArray.msg"
  "msg/ConeMapEntry.msg"
  "msg/ConeMap.msg"
  "msg/PathPoint.msg"
  "msg/PathPointArray.msg"
  "msg/TrackStatus.msg"
  "msg/PoseCorrection.msg"
  "msg/SpeedLimit.msg"
  "msg/VehicleCmd.msg"
  "msg/WheelSpeeds.msg"
  "msg/Heartbeat.msg"
  "msg/VehicleStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
