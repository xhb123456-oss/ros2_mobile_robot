cmake_minimum_required(VERSION 3.8)
project(mobile_robot_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(trajectory_commander src/trajectory_commander.cpp)
ament_target_dependencies(trajectory_commander rclcpp geometry_msgs)

install(TARGETS trajectory_commander DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf worlds rviz config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
