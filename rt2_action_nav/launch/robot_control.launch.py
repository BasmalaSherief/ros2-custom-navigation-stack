import launch
from launch_ros.actions import Node


def generate_launch_description():
    action_server = Node(
        package='rt2_action_nav',
        executable='navigation_action_server_exe',
        name='action_server',
        output='screen',
    )
    
    action_client = Node(
        package='rt2_action_nav',
        executable='UI_action_client_exe',
        name='action_client',
        prefix='xterm -e',  # Launch UI in a new terminal window
        output='screen',
    )

    return launch.LaunchDescription([action_server, action_client])