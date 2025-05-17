cmake_minimum_required(VERSION 3.8)
project(robocore_cpp_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(robocore_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_executable(message_broadcaster src/message_broadcaster.cpp)
ament_target_dependencies(message_broadcaster rclcpp std_msgs)

add_executable(message_listener src/message_listener.cpp)
ament_target_dependencies(message_listener rclcpp std_msgs)

add_executable(config_manager src/config_manager.cpp)
ament_target_dependencies(config_manager rclcpp rcl_interfaces)

add_executable(turtle_motion_controller src/turtle_motion_controller.cpp)
ament_target_dependencies(turtle_motion_controller rclcpp turtlesim)

add_executable(service_provider src/service_provider.cpp)
ament_target_dependencies(service_provider rclcpp robocore_interfaces)

add_executable(service_consumer src/service_consumer.cpp)
ament_target_dependencies(service_consumer rclcpp robocore_interfaces)

add_executable(transform_calculator src/transform_calculator.cpp)
ament_target_dependencies(transform_calculator rclcpp robocore_interfaces geometry_msgs tf2 tf2_ros)

add_executable(lifecycle_manager src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager rclcpp rclcpp_lifecycle std_msgs)

add_executable(qos_message_source src/qos_message_source.cpp)
ament_target_dependencies(qos_message_source rclcpp std_msgs)

add_executable(qos_message_sink src/qos_message_sink.cpp)
ament_target_dependencies(qos_message_sink rclcpp std_msgs)

add_library(task_executor SHARED src/task_executor.cpp)
target_include_directories(task_executor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_executor
  PRIVATE "SIMPLE_ACTION_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(task_executor
  "robocore_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(task_executor 
  PLUGIN "robocore_cpp_demos::SimpleActionServer" 
  EXECUTABLE simple_action_server_node
)

add_library(task_controller SHARED src/task_controller.cpp)
target_include_directories(task_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_controller
  PRIVATE "SIMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(task_controller
  "robocore_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(task_controller 
  PLUGIN "robocore_cpp_demos::SimpleActionClient" 
  EXECUTABLE simple_action_client_node
)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  message_broadcaster
  message_listener
  config_manager
  turtle_motion_controller
  service_provider
  service_consumer
  transform_calculator
  lifecycle_manager
  qos_message_source
  qos_message_sink
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  task_executor
  task_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()