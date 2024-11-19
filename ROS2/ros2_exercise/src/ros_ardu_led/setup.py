from setuptools import setup
import os
from glob import glob

package_name = 'ros_ardu_led'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # launch 파일 복사
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhchoman',
    maintainer_email='jhchoman@todo.todo',
    description='ROS2 package to control Arduino LED through teleop',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_to_arduino = ros_ardu_led.teleop_to_arduino:main',  # teleop_to_arduino 실행 가능하도록 설정
            'teleop_to_console = ros_ardu_led.teleop_to_console:main',
        ],
    },
)




