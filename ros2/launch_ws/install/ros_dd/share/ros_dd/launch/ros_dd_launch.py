import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = os.path.join(
        get_package_share_directory('ros_dd'),
        'urdf/ros_dd.urdf'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path, 'r').read()}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path, 'r').read()}]
    )

    # wheel_controller 노드
    wheel_controller_node = Node(
        package='wheel_controller',
        executable='wheel_controller',  # 위에서 만든 노드 파일명
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        wheel_controller_node  # 자동 바퀴 회전 노드 추가
    ])
