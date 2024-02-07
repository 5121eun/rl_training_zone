import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'training_zone'
    world_filename = 'training_zone.world'
    packge_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(packge_path, "worlds", world_filename)
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
	Node(package=package_name,
             executable='env.py',
             output='screen'
        ),
        Node(package=package_name,
             executable='agent.py',
             output='screen'
        ),
        Node(package=package_name,
             executable='gazebo_service',
             output='screen'
        ),
    ])
