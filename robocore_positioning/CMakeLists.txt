cmake_minimum_required(VERSION 3.8)
project(robocore_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(state_estimator src/state_estimator.cpp)
ament_target_dependencies(state_estimator rclcpp nav_msgs sensor_msgs)

add_executable(sensor_data_relay src/sensor_data_relay.cpp)
ament_target_dependencies(sensor_data_relay rclcpp sensor_msgs)

add_executable(wheel_odometry src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry rclcpp tf2 nav_msgs geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  wheel_odometry
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  state_estimator
  sensor_data_relay
  wheel_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/state_estimator.py
  ${PROJECT_NAME}/sensor_data_relay.py
  ${PROJECT_NAME}/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
