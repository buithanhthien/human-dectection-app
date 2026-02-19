from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        prefix='xterm -e',   # mở terminal riêng
        output='screen'
    )

    return LaunchDescription([
        keyboard_teleop
    ])
