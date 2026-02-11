cmake_minimum_required(VERSION 3.20)
project(roboligo_connector_mavros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(roboligo_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/roboligo_connector_mavros/RoboligoConnectorMavros.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  roboligo_common::roboligo_common
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
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
# endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME})

# Register the control plugins
pluginlib_export_plugin_description_file(roboligo_common roboligo_connector_mavros_plugins.xml)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  roboligo_common
  pluginlib
  geometry_msgs
  lifecycle_msgs
  mavros_msgs
)
ament_package()