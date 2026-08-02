#for github
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
import time


class KinematicsWifiBridge(Node):
    """
    সরলীকৃত (open-loop) ভার্সন - encoder feedback নাই।

    কাজ: /cmd_vel (Twist) সরাসরি পেয়ে kinematics দিয়ে left/right wheel PWM
    বের করে ESP32 কে WiFi (TCP) দিয়ে পাঠায়। কোনো feedback নাই, তাই
    pid_controller.py/odom_publisher.py এখন আর দরকার নাই - এই একটা
    node ই যথেষ্ট real hardware চালানোর জন্য।
    """

    def __init__(self):
        super().__init__('kinematics_wifi_bridge')

        self.declare_parameter('esp_ip', '192.168.1.100')
        self.declare_parameter('esp_port', 8080)
        self.declare_parameter('socket_timeout_sec', 2.0)
        self.declare_parameter('reconnect_interval_sec', 3.0)
        self.declare_parameter('wheel_base', 0.17)     # মিটার - motor বসানোর পর মেপে বসাও
        self.declare_parameter('wheel_radius', 0.03)    # মিটার - চাকার radius
        self.declare_parameter('max_pwm', 200)          # সর্বোচ্চ PWM (255 এর কম রাখলাম, শুরুতে নিরাপদ)
        self.declare_parameter('speed_to_pwm_scale', 40.0)  # rad/s -> PWM conversion factor, tune করে ঠিক করতে হবে

        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value
        self.socket_timeout = self.get_parameter('socket_timeout_sec').value
        self.reconnect_interval = self.get_parameter('reconnect_interval_sec').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.speed_to_pwm_scale = self.get_parameter('speed_to_pwm_scale').value

        self.sock = None
        self.connected = False
        self._lock = threading.Lock()

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.try_connect()
        self.reconnect_timer = self.create_timer(self.reconnect_interval, self.reconnect_check)

        self.get_logger().info("Kinematics WiFi bridge ready (open-loop, no encoder), waiting for /cmd_vel ...")

    def try_connect(self):
        with self._lock:
            try:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.socket_timeout)
                self.sock.connect((self.esp_ip, self.esp_port))
                self.connected = True
                self.get_logger().info(f"Connected with ESP32: {self.esp_ip}:{self.esp_port}")
            except (socket.timeout, OSError) as e:
                self.connected = False
                self.get_logger().error(
                    f"Connection failed! Is the ESP32 turned on and on the same network? Error: {e}"
                )

    def reconnect_check(self):
        if not self.connected:
            self.get_logger().warn("Not connected to ESP32, retrying...")
            self.try_connect()

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # inverse kinematics: robot velocity -> প্রতিটা চাকার angular velocity
        v_r = v + (w * self.wheel_base) / 2.0
        v_l = v - (w * self.wheel_base) / 2.0

        w_r = v_r / self.wheel_radius
        w_l = v_l / self.wheel_radius

        # rad/s -> PWM (এই scale factor motor হাতে পেয়ে টেস্ট করে টিউন করতে হবে,
        # যতটা PWM দিলে motor নড়া শুরু করে সেটাই একটা baseline)
        pwm_l = int(max(-self.max_pwm, min(self.max_pwm, w_l * self.speed_to_pwm_scale)))
        pwm_r = int(max(-self.max_pwm, min(self.max_pwm, w_r * self.speed_to_pwm_scale)))

        command_str = f"{pwm_l},{pwm_r}\n"

        if not self.connected:
            self.get_logger().warn("Skipping send: ESP32 not connected")
            return

        try:
            with self._lock:
                self.sock.sendall(command_str.encode('utf-8'))
            self.get_logger().info(f"Sent PWM: {command_str.strip()}")
        except (socket.timeout, OSError) as e:
            self.get_logger().error(f"There was a problem sending data: {e}")
            self.connected = False

    def destroy_node(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsWifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
