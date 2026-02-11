cmake_minimum_required(VERSION 3.20)
project(roboligo)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(
  roboligo_cli
  roboligo_common
  roboligo_connector
  roboligo_connector_mavros
  roboligo_robot_drone
  roboligo_states
  roboligo_system
)
ament_package()