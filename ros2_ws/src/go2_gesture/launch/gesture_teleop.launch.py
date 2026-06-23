"""Phase 9: gesture_teleop — standalone launch for hand-gesture /cmd_vel.

Drop-in replacement for teleop_twist_keyboard: start this in a new shell
AFTER any robot scene is already running.

Usage (inside the container):
  # Launch any scene first, e.g.:
  ros2 launch go2_description spawn_go2.launch.py
  ros2 launch go2_hunav cafe_go2.launch.py

  # Then in a new shell (./run.sh shell):
  source ~/ros2_ws/install/setup.bash
  ros2 launch go2_gesture gesture_teleop.launch.py

  # Optional overrides:
  ros2 launch go2_gesture gesture_teleop.launch.py camera_device:=1
  ros2 launch go2_gesture gesture_teleop.launch.py max_linear_speed:=0.3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_gesture')
    params_file = os.path.join(pkg, 'config', 'gesture_teleop.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('camera_device',    default_value='0',
                              description='Camera device index (/dev/videoN)'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.4',
                              description='Max forward/backward speed (m/s)'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.8',
                              description='Max turn speed (rad/s)'),

        Node(
            package='go2_gesture',
            executable='gesture_teleop.py',
            name='gesture_teleop',
            output='screen',
            parameters=[
                params_file,
                {
                    'camera_device':     LaunchConfiguration('camera_device'),
                    'max_linear_speed':  LaunchConfiguration('max_linear_speed'),
                    'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                },
            ],
        ),
    ])
