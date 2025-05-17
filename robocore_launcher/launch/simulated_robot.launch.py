import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  use_slam = LaunchConfiguration("use_slam")

  use_slam_arg = DeclareLaunchArgument(
    "use_slam",
    default_value="false"
  )

  gazebo = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("robocore_model"),
      "launch",
      "gazebo.launch.py"
    ),
  )
  
  controller = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("robocore_control"),
      "launch",
      "controller.launch.py"
    ),
    launch_arguments={
      "use_simple_controller": "False",
      "use_python": "False"
    }.items(),
  )
  
  joystick = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("robocore_control"),
      "launch",
      "joystick_teleop.launch.py"
    ),
    launch_arguments={
      "use_sim_time": "True"
    }.items()
  )

  emergency_halt = Node(
    package="robocore_utilities",
    executable="emergency_halt",
    output="screen",
    parameters=[{"use_sim_time": True}]
  )

  localization = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("robocore_positioning"),
      "launch",
      "global_localization.launch.py"
    ),
    condition=UnlessCondition(use_slam)
  )

  slam = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory("robocore_navigation"),
      "launch",
      "slam.launch.py"
    ),
    condition=IfCondition(use_slam)
  )

  rviz_localization = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", os.path.join(
        get_package_share_directory("robocore_positioning"),
        "rviz",
        "global_localization.rviz"
      )
    ],
    output="screen",
    parameters=[{"use_sim_time": True}],
    condition=UnlessCondition(use_slam)
  )

  rviz_slam = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", os.path.join(
        get_package_share_directory("robocore_navigation"),
        "rviz",
        "slam.rviz"
      )
    ],
    output="screen",
    parameters=[{"use_sim_time": True}],
    condition=IfCondition(use_slam)
  )
  
  return LaunchDescription([
    use_slam_arg,
    gazebo,
    controller,
    joystick,
    emergency_halt,
    localization,
    slam,
    rviz_localization,
    rviz_slam
  ])