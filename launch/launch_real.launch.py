from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kinematics_wifi_bridge_node = Node(
        package='ros2_modular_diffbot',
        executable='kinematics_wifi_bridge',
        name='kinematics_wifi_bridge',
        output='screen'
    )

    return LaunchDescription([
        kinematics_wifi_bridge_node
    ])
