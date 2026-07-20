import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class PIDController(Node):
    """
    এই node টাই আসল "brain" - real robot এর জন্য closed-loop speed control করে।
    (Gazebo simulation এ এই node দরকার নাই, কারণ gazebo_ros_diff_drive
     plugin নিজেই cmd_vel অনুযায়ী চাকা ঘোরায়। এই node শুধু real ESP32
     robot চালানোর সময় লঞ্চ করবে।)

    কাজ:
      1) /cmd_vel (Twist) থেকে desired left/right wheel angular velocity বের করা
         (ঠিক এই formula আগে kinematics_wifi_bridge.py তে ছিল)
      2) /wheel_feedback (ESP32 encoder থেকে আসা actual wheel speed) এর সাথে
         desired velocity তুলনা করে PID দিয়ে PWM বের করা
      3) /wheel_pwm এ PWM পাবলিশ করা, যা kinematics_wifi_bridge.py শুনে
         ESP32 কে পাঠায়
    """

    def __init__(self):
        super().__init__('pid_controller')

        # --- robot geometry (gazebo diff_drive plugin এর সাথে অবশ্যই মিলতে হবে) ---
        self.declare_parameter('wheel_base', 0.17)     # wheel_separation, মিটার
        self.declare_parameter('wheel_radius', 0.03)    # মিটার

        # --- PID gains (শুরুতে P শুধু দিয়ে টিউন করা সহজ, তারপর I, D যোগ করো) ---
        self.declare_parameter('kp', 15.0)
        self.declare_parameter('ki', 5.0)
        self.declare_parameter('kd', 0.5)

        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('control_frequency', 10.0)  # Hz - WiFi এর জন্য এর বেশি দরকার নাই

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_pwm = self.get_parameter('max_pwm').value
        control_freq = self.get_parameter('control_frequency').value

        # target (cmd_vel থেকে হিসাব করা desired wheel speed, rad/s)
        self.target_left = 0.0
        self.target_right = 0.0

        # actual (ESP32 feedback থেকে আসা)
        self.actual_left = 0.0
        self.actual_right = 0.0
        self.feedback_received = False

        # PID internal state (প্রতিটা চাকার জন্য আলাদা)
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.feedback_sub = self.create_subscription(
            Float32MultiArray, 'wheel_feedback', self.feedback_callback, 10
        )
        self.pwm_pub = self.create_publisher(Int32MultiArray, 'wheel_pwm', 10)

        self.dt = 1.0 / control_freq
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("PID controller ready (real-hardware only, not needed in Gazebo sim)")

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v_r = v + (w * self.wheel_base) / 2.0
        v_l = v - (w * self.wheel_base) / 2.0

        self.target_right = v_r / self.wheel_radius
        self.target_left = v_l / self.wheel_radius

        # robot থামাতে বললে integral windup যেন আটকে না রাখে সেজন্য রিসেট
        if v == 0.0 and w == 0.0:
            self.integral_left = 0.0
            self.integral_right = 0.0

    def feedback_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn("wheel_feedback message must have exactly 2 values [left, right]")
            return
        self.actual_left, self.actual_right = msg.data[0], msg.data[1]
        self.feedback_received = True

    def _pid_step(self, target, actual, integral, prev_error):
        error = target - actual
        integral += error * self.dt
        derivative = (error - prev_error) / self.dt

        output = (self.kp * error) + (self.ki * integral) + (self.kd * derivative)

        # anti-windup: output clamp করার সাথে সাথে integral ও ক্ল্যাম্প করছি
        output_clamped = max(-self.max_pwm, min(self.max_pwm, output))
        if output != output_clamped:
            integral -= error * self.dt  # windup বাড়তে দিলাম না

        return output_clamped, integral, error

    def control_loop(self):
        if not self.feedback_received:
            # ESP32 থেকে এখনো কোনো feedback আসেনি - PID না চালিয়ে চুপচাপ অপেক্ষা করছি
            return

        pwm_left, self.integral_left, self.prev_error_left = self._pid_step(
            self.target_left, self.actual_left, self.integral_left, self.prev_error_left
        )
        pwm_right, self.integral_right, self.prev_error_right = self._pid_step(
            self.target_right, self.actual_right, self.integral_right, self.prev_error_right
        )

        msg = Int32MultiArray()
        msg.data = [int(pwm_left), int(pwm_right)]
        self.pwm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
