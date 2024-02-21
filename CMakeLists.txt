cmake_minimum_required(VERSION 3.8)
project(simple_mpc_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pacmod3_msgs REQUIRED)

include_directories(
  include
)

add_executable(simple_mpc_publisher src/simple_mpc_publisher.cpp ${trajectory_planner_sources})
ament_target_dependencies(simple_mpc_publisher rclcpp geometry_msgs tf2_geometry_msgs tf2 tf2_ros nav_msgs pacmod3_msgs autoware_adapi_v1_msgs autoware_auto_planning_msgs autoware_auto_vehicle_msgs visualization_msgs)
target_link_libraries(simple_mpc_publisher "${cpp_typesupport_target}")


install(TARGETS
  simple_mpc_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()