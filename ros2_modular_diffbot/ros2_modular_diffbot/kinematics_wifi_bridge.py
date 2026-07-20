#for github
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import socket
import threading
import time


class KinematicsWifiBridge(Node):
    """
    এই node টা শুধু ESP32 এর সাথে WiFi যোগাযোগ সামলায়।
    কোনো kinematics/PID এখানে নাই ইচ্ছাকৃতভাবে - সেটা pid_controller.py করে।
    এই node এর কাজ দুইটা:
      1) /wheel_pwm topic থেকে PWM পেয়ে ESP32 কে পাঠানো (TX)
      2) ESP32 থেকে encoder feedback পড়ে /wheel_feedback topic এ পাবলিশ করা (RX)

    ESP32 protocol (দুইদিকেই plain text, newline দিয়ে আলাদা):
      Pi -> ESP32   :  "<left_pwm>,<right_pwm>\n"          (int)
      ESP32 -> Pi   :  "<left_rad_s>,<right_rad_s>\n"       (float, actual wheel speed)
    """

    def __init__(self):
        super().__init__('kinematics_wifi_bridge')

        self.declare_parameter('esp_ip', '192.168.1.100')
        self.declare_parameter('esp_port', 8080)
        self.declare_parameter('socket_timeout_sec', 2.0)
        self.declare_parameter('reconnect_interval_sec', 3.0)

        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value
        self.socket_timeout = self.get_parameter('socket_timeout_sec').value
        self.reconnect_interval = self.get_parameter('reconnect_interval_sec').value

        self.sock = None
        self.connected = False
        self._lock = threading.Lock()  # sock ব্যবহারে TX/RX থ্রেড একসাথে না পড়ার জন্য

        self.feedback_pub = self.create_publisher(Float32MultiArray, 'wheel_feedback', 10)

        self.subscription = self.create_subscription(
            Int32MultiArray, 'wheel_pwm', self.wheel_pwm_callback, 10
        )

        # প্রথম connection চেষ্টা
        self.try_connect()

        # connection ছুটে গেলে বারবার নিজে নিজে retry করার জন্য timer
        self.reconnect_timer = self.create_timer(self.reconnect_interval, self.reconnect_check)

        # ESP32 থেকে ব্লকিং recv() আলাদা থ্রেডে চালাচ্ছি, নাহলে rclpy spin আটকে যাবে
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info("Kinematics WiFi bridge ready, waiting for /wheel_pwm ...")

    # ---------------- Connection handling ----------------

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

    # ---------------- TX: cmd -> ESP32 ----------------

    def wheel_pwm_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn("wheel_pwm message must have exactly 2 values [left, right]")
            return

        pwm_l, pwm_r = int(msg.data[0]), int(msg.data[1])
        command_str = f"{pwm_l},{pwm_r}\n"

        if not self.connected:
            self.get_logger().warn("Skipping send: ESP32 not connected")
            return

        try:
            with self._lock:
                self.sock.sendall(command_str.encode('utf-8'))
        except (socket.timeout, OSError) as e:
            self.get_logger().error(f"There was a problem sending data: {e}")
            self.connected = False

    # ---------------- RX: ESP32 -> feedback topic ----------------

    def receive_loop(self):
        buffer = ""
        while rclpy.ok():
            if not self.connected or self.sock is None:
                time.sleep(0.2)
                continue
            try:
                with self._lock:
                    self.sock.settimeout(self.socket_timeout)
                    data = self.sock.recv(1024)
                if not data:
                    raise ConnectionError("ESP32 closed the connection")

                buffer += data.decode('utf-8', errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_and_publish_feedback(line.strip())

            except socket.timeout:
                continue  # normal - কোনো ডেটা আসেনি এই মুহূর্তে, retry করো
            except (OSError, ConnectionError) as e:
                self.get_logger().error(f"Lost connection while reading: {e}")
                self.connected = False
                time.sleep(0.2)

    def parse_and_publish_feedback(self, line: str):
        try:
            left_str, right_str = line.split(',')
            left_rad_s = float(left_str)
            right_rad_s = float(right_str)
        except (ValueError, IndexError):
            self.get_logger().warn(f"Malformed feedback from ESP32, ignoring: '{line}'")
            return

        msg = Float32MultiArray()
        msg.data = [left_rad_s, right_rad_s]
        self.feedback_pub.publish(msg)

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
