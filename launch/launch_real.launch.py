import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Real ESP32 hardware চালানোর জন্য launch file - Gazebo/RViz কিছুই লাগে না।

    এই তিনটা node একসাথে চালু হবে:
      1) kinematics_wifi_bridge - ESP32 এর সাথে WiFi (TCP) communication
      2) pid_controller          - cmd_vel + encoder feedback দিয়ে PWM বের করে
      3) odom_publisher          - encoder feedback দিয়ে position/TF বের করে

    NOTE: এটা Gazebo simulation এর সাথে একসাথে চালিও না -
    launch_sim.launch.py এর gazebo_ros_diff_drive plugin ইতিমধ্যে
    /odom + TF publish করে, দুইটা একসাথে চললে conflict হবে।
    """
    package_name = 'ros2_modular_diffbot'

    pkg_path = get_package_share_directory(package_name)
    pid_params_file = os.path.join(pkg_path, 'config', 'pid_params.yaml')

    kinematics_wifi_bridge_node = Node(
        package=package_name,
        executable='kinematics_wifi_bridge',
        name='kinematics_wifi_bridge',
        output='screen'
        # esp_ip/esp_port ইত্যাদি ডিফল্ট প্যারামিটার দিয়েই চলবে
        # (kinematics_wifi_bridge.py এর ভেতরে declare_parameter এ যা আছে)
    )

    pid_controller_node = Node(
        package=package_name,
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[pid_params_file]
    )

    odom_publisher_node = Node(
        package=package_name,
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[pid_params_file]
    )

    return LaunchDescription([
        kinematics_wifi_bridge_node,
        pid_controller_node,
        odom_publisher_node
    ])
