from setuptools import setup

package_name = 'wheel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wheel_controller_launch.py']),
        # ('share/' + package_name + '/rviz', ['rviz/wheel_controller.rviz']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jang',
    maintainer_email='jang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wheel_controller = wheel_controller.wheel_controller:main",
        ],
    },
)
