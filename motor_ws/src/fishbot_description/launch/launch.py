from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 启动motor_controller_node
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
        ),
        # 启动控制器管理器
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                FindPackageShare('your_robot_description').find('your_robot_description') + '/config/robot.urdf',
                FindPackageShare('your_controller_package').find('your_controller_package') + '/config/diff_drive_controller.yaml'
            ],
            output='screen',
        ),
        # 启动差速控制器
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['diff_drive_controller'],
        )
    ])
