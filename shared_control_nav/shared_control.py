import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SharedControlNode(Node):

    def __init__(self):
        super().__init__('shared_control')

        self.declare_parameter('control_mode', 'assisted')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('warning_radius', 1.25)
        self.declare_parameter('hard_radius', 1.5)

        self.declare_parameter('recovery_radius', 0.75)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.05)
        self.declare_parameter('recovery_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.35)

        self.front_distance = None
        self.rear_distance = None
        self.last_teleop_cmd = None

        self.control_mode = self.get_parameter('control_mode').value
        self.last_status = {}

        self.home_x = None
        self.home_y = None

        self.current_x = None
        self.current_y = None
        self.yaw = None

        self.safe_points = []
        self.recovery_target = None

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/teleop_cmd_vel',
            self.teleop_callback,
            10
        )

        self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/shared_cmd_vel',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/shared_control_status',
            10
        )

        self.teleop_reset_pub = self.create_publisher(
            String,
            '/teleop_reset',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_command
        )

    # Read front and rear LiDAR sectors for assisted safety checks.
    def scan_callback(self, msg):
        front_readings = list(msg.ranges[0:6]) + list(msg.ranges[355:360])
        rear_readings = list(msg.ranges[175:186])

        valid_front = self.valid_readings(front_readings)
        valid_rear = self.valid_readings(rear_readings)

        if len(valid_front) > 0:
            self.front_distance = min(valid_front)
        else:
            self.front_distance = float('inf')
        
        if len(valid_rear) > 0:
            self.rear_distance = min(valid_rear)
        else:            
            self.rear_distance = float('inf')

    # Track the robot's current odometry position and save the start position as home.
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

        if self.home_x is None or self.home_y is None:
            self.home_x = self.current_x
            self.home_y = self.current_y

            self.generate_safe_points()
            self.publish_status('boundary', 'Boundary: home set')

    def teleop_callback(self, msg):
        self.last_teleop_cmd = msg

    def control_mode_callback(self, msg):
        if msg.data == 'manual' or msg.data == 'assisted' or msg.data == 'recovery':
            self.control_mode = msg.data
            self.publish_status('mode', 'Mode: ' + self.control_mode)

            if self.control_mode == 'recovery':
                self.recovery_target = None

        else:
            self.publish_status('mode', 'Mode: ' + msg.data + ' is unknown')

    # Main control loop. Chooses behavior based on the current mode.
    def publish_command(self):
        mode = self.control_mode

        if mode == 'recovery':
            cmd = self.apply_recovery_control()
            cmd = self.apply_safety_filter(cmd)
            self.cmd_pub.publish(cmd)
            return

        if self.last_teleop_cmd is None:
            return

        cmd = self.copy_cmd(self.last_teleop_cmd)

        if mode == 'assisted':
            cmd = self.apply_safety_filter(cmd)
            cmd = self.apply_boundary_filter(cmd)

        elif mode == 'manual':
            self.publish_status('mode', 'Mode: manual')

        else:
            self.get_logger().warn('Unknown control mode: ' + mode)
            self.publish_status('mode', 'Mode: ' + mode + ' is unknown')
            cmd = self.make_stop_cmd()

        self.cmd_pub.publish(cmd)

    # Block unsafe forward or backward motion when an obstacle is too close.
    def apply_safety_filter(self, cmd):
        stop_distance = self.get_parameter('stop_distance').value

        if self.front_distance is None or self.rear_distance is None:
            if cmd.twist.linear.x != 0.0:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0

            self.publish_status('safety', 'Safety: waiting for scan')
            return cmd

        if self.front_distance < stop_distance and cmd.twist.linear.x > 0.0:
            cmd.twist.linear.x = 0.0
            self.publish_status('safety', 'Safety: forward blocked')
            return cmd
        
        if self.rear_distance < stop_distance and cmd.twist.linear.x < 0.0:
            cmd.twist.linear.x = 0.0
            self.publish_status('safety', 'Safety: rear blocked')
            return cmd
            
        self.publish_status('safety', 'Safety: clear')
        return cmd

    # Monitor distance from home and start recovery if the robot reaches the boundary.
    def apply_boundary_filter(self, cmd):
        distance = self.distance_from_home()

        if distance is None:
            self.publish_status('boundary', 'Boundary: waiting for odom')
            return cmd

        warning_radius = self.get_parameter('warning_radius').value
        hard_radius = self.get_parameter('hard_radius').value

        if distance >= hard_radius:
            self.publish_status('boundary', 'Boundary: limit reached')
            self.start_recovery_mode()
            return self.make_stop_cmd()
            
        elif distance >= warning_radius:
            self.publish_status('boundary', 'Boundary: warning')

        else:
            self.publish_status('boundary', 'Boundary: clear')

        return cmd

    # Drive toward the nearest safe recovery point using odometry.
    def apply_recovery_control(self):
        if self.current_x is None or self.current_y is None or self.yaw is None:
            self.publish_status('recovery', 'Recovery: waiting for odom')
            return self.make_stop_cmd()

        if self.recovery_target is None:
            self.recovery_target = self.select_recovery_target()

            if self.recovery_target is None:
                self.publish_status('recovery', 'Recovery: no target')
                return self.make_stop_cmd()

            self.publish_status('recovery', 'Recovery: target selected')

        target_x = self.recovery_target[0]
        target_y = self.recovery_target[1]

        distance = self.find_distance(
            self.current_x,
            target_x,
            self.current_y,
            target_y
        )

        goal_tolerance = self.get_parameter('goal_tolerance').value

        if distance <= goal_tolerance:
            self.recovery_target = None
            self.control_mode = 'assisted'
            self.last_teleop_cmd = self.make_stop_cmd()
            self.reset_teleop()

            self.publish_status('mode', 'Mode: assisted')
            self.publish_status('recovery', 'Recovery: complete')

            return self.make_stop_cmd()

        target_angle = math.atan2(
            target_y - self.current_y,
            target_x - self.current_x
        )   

        angle_error = self.normalize_angle(target_angle - self.yaw)

        cmd = self.make_stop_cmd()

        heading_tolerance = self.get_parameter('heading_tolerance').value
        linear_speed = self.get_parameter('recovery_linear_speed').value
        angular_speed = self.get_parameter('recovery_angular_speed').value

        if abs(angle_error) > heading_tolerance:
            if angle_error > 0.0:
                cmd.twist.angular.z = angular_speed
            else:
                cmd.twist.angular.z = -angular_speed

            self.publish_status('recovery', 'Recovery: turning')

        else:
            cmd.twist.linear.x = linear_speed
            self.publish_status('recovery', 'Recovery: driving')

        return cmd

    # Switch into recovery mode and clear stored teleop command.
    def start_recovery_mode(self):
        self.control_mode = 'recovery'
        self.recovery_target = None
        self.last_teleop_cmd = self.make_stop_cmd()
        self.reset_teleop()

        self.publish_status('mode', 'Mode: recovery')
        self.publish_status('recovery', 'Recovery: activated')

    # Create recovery points around the starting position.
    def generate_safe_points(self):
        recovery_radius = self.get_parameter('recovery_radius').value

        self.safe_points = [
            (self.home_x + recovery_radius, self.home_y),
            (self.home_x - recovery_radius, self.home_y),
            (self.home_x, self.home_y + recovery_radius),
            (self.home_x, self.home_y - recovery_radius),
        ]

    # Choose the recovery point closest to the robot's current position.
    def select_recovery_target(self):
        if len(self.safe_points) == 0:
            return None

        closest_point = None
        closest_distance = None

        for point in self.safe_points:
            distance = self.find_distance(
                self.current_x,
                point[0],
                self.current_y,
                point[1]
            )

            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_point = point

        return closest_point

    def distance_from_home(self):
        if self.home_x is None or self.home_y is None:
            return None

        if self.current_x is None or self.current_y is None:
            return None

        return self.find_distance(
            self.current_x,
            self.home_x,
            self.current_y,
            self.home_y
        )

    def valid_readings(self, readings):
        valid = []

        for reading in readings:
            if math.isfinite(reading) and reading > 0.0:
                valid.append(reading)

        return valid

    def find_distance(self, x1, x0, y1, y0):
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def copy_cmd(self, msg):
        new_cmd = TwistStamped()
        new_cmd.header.stamp = self.get_clock().now().to_msg()
        new_cmd.header.frame_id = 'base_link'

        new_cmd.twist.linear.x = msg.twist.linear.x
        new_cmd.twist.linear.y = msg.twist.linear.y
        new_cmd.twist.linear.z = msg.twist.linear.z
        new_cmd.twist.angular.x = msg.twist.angular.x
        new_cmd.twist.angular.y = msg.twist.angular.y
        new_cmd.twist.angular.z = msg.twist.angular.z

        return new_cmd

    def make_stop_cmd(self):
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = 'base_link'

        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.angular.z = 0.0

        return stop_cmd

    def stop(self):
        self.cmd_pub.publish(self.make_stop_cmd())

    def publish_status(self, category, status):
        if self.last_status.get(category) != status:
            msg = String()
            msg.data = status
            self.status_pub.publish(msg)
            self.last_status[category] = status

    # Tell the teleop node to clear its stored speed values.
    def reset_teleop(self):
        msg = String()
        msg.data = 'reset'
        self.teleop_reset_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = SharedControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.stop()
            time.sleep(0.5)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()