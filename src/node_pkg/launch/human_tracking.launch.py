from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    workspace_root = '/home/thien/human_detection_app'
    venv_path = os.path.join(workspace_root, '.venv', 'lib', 'python3.12', 'site-packages')
    
    return LaunchDescription([
        SetEnvironmentVariable('HUMAN_DETECTION_APP_ROOT', workspace_root),
        SetEnvironmentVariable('PYTHONPATH', f"{venv_path}:{os.environ.get('PYTHONPATH', '')}"),
        Node(
            package='node_pkg',
            executable='human_tracking_node',
            name='human_tracking_node',
            output='screen'
        )
    ])
