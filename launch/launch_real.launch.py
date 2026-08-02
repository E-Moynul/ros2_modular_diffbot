from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Real ESP32 hardware চালানোর জন্য launch file (simplified, no encoder/PID/odom)।
    শুধু একটাই node - kinematics_wifi_bridge - যেটা cmd_vel সরাসরি PWM এ
    কনভার্ট করে ESP32 কে পাঠায়।
    """
    kinematics_wifi_bridge_node = Node(
        package='ros2_modular_diffbot',
        executable='kinematics_wifi_bridge',
        name='kinematics_wifi_bridge',
        output='screen'
    )

    return LaunchDescription([
        kinematics_wifi_bridge_node
    ])
