from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_control = Node(
        package='speed',
        executable='motor_control',
        name='motor_control',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    xbox_interface = Node(
        package='teleop',
        executable='xbox_control_interface',
        name='xbox_control_interface',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        motor_control,
        xbox_interface,
    ])