cmake_minimum_required(VERSION 3.20)
project(roboligo_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(roboligo_common REQUIRED)
find_package(roboligo_states REQUIRED)
find_package(roboligo_connector REQUIRED)
find_package(roboligo_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)


set(dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  roboligo_common::roboligo_common
  roboligo_states::roboligo_states
  roboligo_connector::roboligo_connector
  roboligo_interfaces::roboligo_interfaces
)

set(package_dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  roboligo_common
  roboligo_states
  roboligo_connector
  roboligo_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/roboligo_system/RoboligoNode.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  roboligo_common::roboligo_common
  roboligo_states::roboligo_states
  roboligo_connector::roboligo_connector
  ${roboligo_interfaces_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${mavros_msgs_TARGETS}
)

add_executable(roboligo_main src/roboligo_main.cpp)
target_link_libraries(roboligo_main PUBLIC
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  roboligo_main
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
  roboligo_common
  roboligo_states
  roboligo_connector
  roboligo_interfaces
  mavros_msgs
)
ament_package()