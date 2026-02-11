cmake_minimum_required(VERSION 3.20)
project(roboligo_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/roboligo_common/types/RobotState.cpp
  src/roboligo_common/classification/Mode.cpp
  src/roboligo_common/classification/Trigger.cpp
  src/roboligo_common/classification/ClassificationBase.cpp
  src/roboligo_common/connector/Sensor.cpp
  src/roboligo_common/connector/Commander.cpp
  src/roboligo_common/connector/commanders/Input.cpp
  src/roboligo_common/connector/commanders/Output.cpp
  src/roboligo_common/connector/commanders/Service.cpp
  src/roboligo_common/connector/sensors/Battery.cpp
  src/roboligo_common/connector/sensors/Imu.cpp
  src/roboligo_common/connector/sensors/NavSat.cpp
  src/roboligo_common/connector/sensors/Odom.cpp
  src/roboligo_common/connector/ConnectorBase.cpp
  src/roboligo_common/types/Interface.cpp
  src/roboligo_common/types/Linker.cpp
  src/roboligo_common/types/PluginBase.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(tests)
# endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  sensor_msgs
  nav_msgs
  mavros_msgs
)
ament_package()