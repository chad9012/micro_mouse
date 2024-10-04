from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the paths for the URDF and RViz config
    urdf_path = os.path.join(
        get_package_share_directory('micro_mouse_description'),
        'urdf',
        'micro_mouse.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('micro_mouse_bringup'),
        'rviz',
        'urdf_config.rviz'
    )

    return LaunchDescription([
        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ))
        ,

        # Node for the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),

        # Node for spawning the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'micro_mouse'],
            output='screen'
        ),

        # Node for launching RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]  # Changed to use -d for the RViz config
        ),
    ])
