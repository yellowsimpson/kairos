'''
/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('jdamr_ros_03'), 'urdf', 'jdamr.urdf')
    #rviz_config = os.path.join(get_package_share_directory('jdcobot_100_description'), 'rviz', 'rviz_set.rviz')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',),
        
			Node(
				package='robot_state_publisher',
				executable='robot_state_publisher',
				name='robot_state_publisher',
				output='screen',
				parameters=[{
				'use_sim_time': use_sim_time,
				'robot_description': robot_desc,
				'source_list': ['joint1', 'joint2', 'joint3', 'joint4'],
				'zeros': {
					'joint1': 1.5708,
					'joint2': 1.5708,
					'joint3': 1.5708,
					'joint4': 1.5708
				}
				}]
			),
			Node(
				package='jdamr_ros_03',
				executable='jdamr_03',
				name='jdamr_03',
				output='screen'
			
			),
			Node(
				package='rviz2',
				executable='rviz2',
				name='rviz2',
				output='screen',
				arguments=['-d', os.path.join(get_package_share_directory('jdamr_ros_03'),'rviz','jdamr_rviz.rviz')]
			),
   		])


if __name__ == '__main__':
    print(os.path.join(get_package_share_directory('jdam_ros_03'), 'urdf', 'jdamr.urdf'))
    generate_launch_description()
