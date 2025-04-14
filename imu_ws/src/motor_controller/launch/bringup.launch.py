from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('motor_controller'),
        'urdf',
        'yj_bot.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    controller_config = os.path.join(
        get_package_share_directory('motor_controller'),
        'config',
        'diff_drive_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_config
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
        Node(
            package='motor_controller',
            executable='controller_bridge_node',
            output='screen',
        ),
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            output='screen',
        )
    ])