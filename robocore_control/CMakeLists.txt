cmake_minimum_required(VERSION 3.5)
project(robocore_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(motion_controller src/motion_controller.cpp)
ament_target_dependencies(motion_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(filtered_controller src/filtered_controller.cpp)
ament_target_dependencies(filtered_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)

add_executable(velocity_dispatcher src/velocity_dispatcher.cpp)
ament_target_dependencies(velocity_dispatcher rclcpp geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  motion_controller
  filtered_controller
  velocity_dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/motion_controller.py
  ${PROJECT_NAME}/filtered_controller.py
  ${PROJECT_NAME}/velocity_dispatcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()