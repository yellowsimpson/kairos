from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 및 파일 이름 설정
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('ros_ardu_led'), 'urdf', 'ros_ardu_led.urdf')

    # URDF 파일 읽기
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
                
                'robot_description': robot_description,
                'source_list': ['wheel1_joint', 'wheel2_joint', 'wheel3_joint', 'wheel4_joint'],
                'zeros': {
                    'wheel1_joint':1.5708,
                    'wheel2_joint':1.5708,
                    'wheel3_joint':1.5708,
                    'wheel4_joint':1.5708
                }                
            }]
    )
    

    joint_state_publisher_node=Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'source_list': ['wheel1_joint', 'wheel2_joint', 'wheel3_joint', 'wheel4_joint'],
                'zeros': {
                    'wheel1_joint':1.5708,
                    'wheel2_joint':1.5708,
                    'wheel3_joint':1.5708,
                    'wheel4_joint':1.5708
                }                
            }]
        ),


    # RViz 노드 (기본 설정으로 실행)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('ros_ardu_led'),'rviz','ros_ardu_led_rviz.rviz')] # Replace with your RViz config file path
    )

    # arduino_commander 노드 (패키지 내에 존재하는지 확인)
    arduino_commander_node = Node(
        package='ros_ardu_led',
        executable='arduino_commander',
        name='arduino_commander'
    )

    # # Teleop Twist Keyboard 노드
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     prefix='gnome-terminal --',
    #     remappings=[('/cmd_vel', '/cmd_vel')]
    # )

    return LaunchDescription([
        arduino_commander_node,
        robot_state_publisher_node,
        rviz_node,
        # teleop_node,
    ])

if __name__ =='__main__':
    generate_launch_description()
