from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():

    kinematics_wifi_bridge_node = Node(
        package='ros2_modular_diffbot',
        executable='kinematics_wifi_bridge',
        name='kinematics_wifi_bridge',
        output='screen'
    )

    display_teleop_cmd = LogInfo(
        msg="\n\n"
            "========================================================================================\n"
            " Open a new terminal and copy-paste this exact command for perfect tested speed:\n\n"
            " ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.30 -p turn:=3.45\n"
            "========================================================================================\n"
    )

    return LaunchDescription([
        display_teleop_cmd,
        kinematics_wifi_bridge_node
    ])
