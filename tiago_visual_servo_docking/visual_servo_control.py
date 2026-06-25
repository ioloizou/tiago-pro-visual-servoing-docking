import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import String


def get_camera_yaw(q):
    """
    Panning yaw of the tag about the camera's vertical axis, on the X-Z floor plane.
    Uses the tag's X-axis as the facing direction (the X-Axis Hack). When the tag
    faces the camera head-on, the facing axis points back at the camera (-Z_cam),
    so this returns 0.
    """
    nx = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)   # tag X-axis, X component (camera frame)
    nz = 2.0 * (q.x * q.z - q.w * q.y)         # tag X-axis, Z component (camera frame)
    return math.atan2(nx, -nz)                 # head-on -> atan2(0, 1) = 0


class VisualServoControlNode(Node):
    def __init__(self):
        super().__init__('visual_servo_control_node')

        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('kp_linear', 0.15)
        self.declare_parameter('kp_angular', 0.3)
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.3)
        # Target pose expressed as an offset from the AprilTag frame.
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('april_tag_timeout', 1.0)

        april_tag_pose_topic = '/apriltag/pose'
        cmd_vel_topic = '/cmd_vel'
        docking_status_topic = '/docking_status'

        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_yaw = self.get_parameter('target_yaw').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.april_tag_timeout = self.get_parameter('april_tag_timeout').value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.april_tag_pose: Optional[Pose] = None
        self.last_april_tag_stamp = None
        self.docked = False

        self.april_tag_sub = self.create_subscription(
            PoseStamped, april_tag_pose_topic, self.april_tag_pose_callback, sensor_qos)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, docking_status_topic, 10)

        control_period = 1.0 / self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info('Visual servo control node started (Omnidirectional with X-Axis Hack).')

    def april_tag_pose_callback(self, msg):
        self.april_tag_pose = msg.pose
        self.last_april_tag_stamp = self.get_clock().now()

    def april_tag_is_stale(self):
        if self.last_april_tag_stamp is None:
            return True
        elapsed = (self.get_clock().now() - self.last_april_tag_stamp).nanoseconds / 1e9
        return elapsed > self.april_tag_timeout

    def compute_error(self) -> tuple[float, float, float]:
        assert self.april_tag_pose is not None
        p = self.april_tag_pose.position
        q = self.april_tag_pose.orientation

        # Camera Z is forward (Robot X). Camera X is right (Robot Y is left).
        error_x = p.z - self.target_x
        error_y = -p.x + self.target_y

        # Yaw error normalized to [-pi, pi]
        current_yaw = get_camera_yaw(q)
        yaw_error = current_yaw - self.target_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        return error_x, error_y, yaw_error

    def compute_control(self, error_x, error_y, yaw_error):
        twist = Twist()

        # Only return True if BOTH position AND yaw are perfect at the exact same time
        if abs(error_x) <= self.position_tolerance and abs(error_y) <= self.position_tolerance and abs(yaw_error) <= self.yaw_tolerance:
            self.get_logger().info('DOCKED!')
            return twist, True

        # Always calculate all three velocities continuously
        twist.linear.x = self.clamp(self.kp_linear * error_x, self.max_linear_vel)
        twist.linear.y = self.clamp(self.kp_linear * error_y, self.max_linear_vel)
        twist.angular.z = self.clamp(self.kp_angular * yaw_error, self.max_angular_vel)

        self.get_logger().info(
            'ALIGNING: err_x=%.4f err_y=%.4f yaw_err=%.4f cmd_vx=%.4f cmd_vy=%.4f cmd_wz=%.4f' % (
                error_x, error_y, yaw_error, twist.linear.x, twist.linear.y, twist.angular.z))

        return twist, False

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def control_loop(self):
        if self.april_tag_pose is None or self.april_tag_is_stale():
            self.stop_robot()
            self.publish_status('SEARCHING')
            self.get_logger().warn('SEARCHING: no AprilTag data (never received or stale)',
                                   throttle_duration_sec=2.0)
            self.docked = False
            return

        error_x, error_y, yaw_error = self.compute_error()
        twist, docked = self.compute_control(error_x, error_y, yaw_error)

        if docked:
            self.stop_robot()
            self.publish_status('DOCKED')
            self.docked = True
            return

        self.docked = False
        self.cmd_vel_pub.publish(twist)
        self.publish_status('ALIGNING')


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()