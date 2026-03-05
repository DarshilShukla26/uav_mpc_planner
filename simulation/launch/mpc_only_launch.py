from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    mpc_node = Node(
        package='uav_mpc',
        executable='mpc_node', # Requires ament_python build in real workspace
        name='mpc_node',
        output='screen',
        parameters=[{'frequency': 50.0, 'horizon': 20}]
    )

    return LaunchDescription([
        mpc_node
    ])
