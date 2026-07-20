import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):
    """
    Real robot এর জন্য odometry - ESP32 encoder থেকে আসা actual wheel speed
    (/wheel_feedback) কে integrate করে robot এর position (x, y, theta) বের করে,
    এবং /odom topic + odom->base_link TF পাবলিশ করে।

    IMPORTANT: Gazebo simulation এ এই node চালিও না। gazebo_ros_diff_drive
    plugin (robot_gazebo.xacro) ইতিমধ্যে publish_odom_tf=true দিয়ে
    odom->base_link TF নিজেই দিচ্ছে। দুইজনে একসাথে চালালে TF conflict হবে।
    """

    def __init__(self):
        super().__init__('odom_publisher')

        self.declare_parameter('wheel_base', 0.17)
        self.declare_parameter('wheel_radius', 0.03)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # robot pose state (odom frame এ)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = None

        self.feedback_sub = self.create_subscription(
            Float32MultiArray, 'wheel_feedback', self.feedback_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odom publisher ready (real-hardware only, not needed in Gazebo sim)")

    def feedback_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn("wheel_feedback message must have exactly 2 values [left, right]")
            return

        left_rad_s, right_rad_s = msg.data[0], msg.data[1]
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return  # প্রথম sample এ dt জানি না, শুধু state init করে রাখলাম

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # forward kinematics: wheel angular velocity -> robot linear/angular velocity
        v = self.wheel_radius * (right_rad_s + left_rad_s) / 2.0
        w = self.wheel_radius * (right_rad_s - left_rad_s) / self.wheel_base

        # 2nd-order (মিডপয়েন্ট) integration - শুধু plain Euler থেকে বেশি নির্ভুল
        delta_theta = w * dt
        theta_mid = self.theta + delta_theta / 2.0

        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # -pi..pi এ রাখা

        self.publish_odom(now, v, w)

    def publish_odom(self, stamp: Time, v: float, w: float):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
