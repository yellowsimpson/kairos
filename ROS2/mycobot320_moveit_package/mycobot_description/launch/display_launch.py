import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('mycobot_description'), 'urdf', 'mycobot_320_gripper.urdf')
    rviz_config = os.path.join(get_package_share_directory('mycobot_description'), 'rviz', 'rviz_set.rviz')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            }],
            arguments=[urdf],
        ),
	Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			parameters=[{
				'use_sim_time': use_sim_time,
				'robot_description': robot_desc,
				
	    	}],
        	arguments=[urdf]
		),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[rviz_config])
    ])


if __name__ == '__main__':
    generate_launch_description()
