import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',),
        
			Node(
				package='ros_cv',
				executable='cam_pub',
				name='cam_pub',				
			),
			Node(
				package='ros_cv',
				executable='cam_sub',
				name='cam_sub',
				output='screen'
			),
			
   		])


if __name__ == '__main__':
    generate_launch_description()
