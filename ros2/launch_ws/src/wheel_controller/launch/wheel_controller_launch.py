import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF 파일 경로 설정
    urdf_file = os.path.join(
        get_package_share_directory('wheel_controller'), 'urdf', 'wheel_controller.urdf'
    )

    return LaunchDescription([
        # 로봇 상태 퍼블리셔를 사용하여 URDF 파일을 퍼블리시합니다.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),
        # RViz를 실행하고 로봇 모델을 시각화합니다.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        # wheel_controller 노드를 실행합니다.
        Node(
            package='wheel_controller',
            executable='wheel_controller',
            name='wheel_controller',
            output='screen'
        ),
    ])
