import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_modular_diffbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rifat',
    maintainer_email='rifat@todo.todo',
    description='ROS2 differential drive robot with Gazebo simulation and ESP32 WiFi bridge for sim-to-real deployment',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'kinematics_wifi_bridge = ros2_modular_diffbot.kinematics_wifi_bridge:main',
        'pid_controller = ros2_modular_diffbot.pid_controller:main',
        'odom_publisher = ros2_modular_diffbot.odom_publisher:main',
        ],
    },
)


