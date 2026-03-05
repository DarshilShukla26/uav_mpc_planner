import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Parameters
    world_idx = LaunchConfiguration('world_idx', default='1')

    # Since this is a placeholder standalone workspace without ament build,
    # paths might be relative in development. 
    # Usually we would find the pkg with: pkg_share = get_package_share_directory('uav_mpc_planner')
    
    mpc_node = Node(
        package='uav_mpc',
        executable='mpc_node', # Note: requires python setup.py entry points in real deployment
        name='mpc_node',
        output='screen',
        parameters=[{'frequency': 50.0, 'horizon': 20}]
    )

    state_estimator = Node(
        package='uav_mpc',
        executable='state_estimator_node',
        name='state_estimator',
        output='screen'
    )
    
    visualizer = Node(
        package='uav_mpc',
        executable='visualizer_node',
        name='visualizer',
        output='screen'
    )

    # Note: A real launch file would IncludeLaunchDescription for gazebo_ros
    # For now, it scaffolds the core nodes needed to orchestrate the pipeline.
    
    return LaunchDescription([
        DeclareLaunchArgument('world_idx', default_value='1', description='Gazebo world index (1-12)'),
        mpc_node,
        state_estimator,
        visualizer
    ])
