# setup.py
from setuptools import find_packages, setup
import os

package_name = 'ros_dd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 리소스 인덱스 설정
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 패키지 설정 파일 포함
        ('share/' + package_name, ['package.xml']),
        # URDF 파일 포함
        (os.path.join('share', package_name, 'urdf'), ['urdf/ros_dd.urdf']),
        # 런치 파일 포함
        (os.path.join('share', package_name, 'launch'), ['launch/rosdd_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhchoman',
    maintainer_email='jhchoman@example.com',
    description='URDF를 RViz에서 시각화하고 teleop을 통해 바퀴를 제어하는 패키지입니다.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_controller = ros_dd.wheel_controller:main',
            'arduino_commander = ros_dd.arduino_commander:main',
        ],
    },
)
