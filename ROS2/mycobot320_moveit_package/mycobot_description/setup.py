import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
from ament_index_python.packages import get_package_share_directory

package_name = 'mycobot_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display_launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_launch.py']),
        ('share/' + package_name + '/launch', ['launch/test_1_launch.py']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'js_pub_test = mycobot_description.js_pub_test:main',
        ],
    },
)
