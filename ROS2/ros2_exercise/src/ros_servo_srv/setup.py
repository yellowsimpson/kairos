from setuptools import find_packages, setup

package_name = 'ros_servo_srv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhchoman',
    maintainer_email='jhchoman@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ser=ros_servo_srv.ser:main',
            'cli=ros_servo_srv.cli:main'
        ],
    },
)