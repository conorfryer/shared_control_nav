import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


MSG = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

Shared-control modes:
1 : manual mode
2 : assisted mode
3 : recovery mode

CTRL-C to quit
"""


class SharedTeleopNode(Node):

    def __init__(self):
        super().__init__('shared_teleop')

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.linear_step = 0.02
        self.angular_step = 0.10

        self.max_linear_speed = 0.22
        self.max_angular_speed = 1.82

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/teleop_cmd_vel',
            10
        )

        self.mode_pub = self.create_publisher(
            String,
            '/control_mode',
            10
        )

        # Listen for reset messages from shared_control after automatic recovery.
        self.create_subscription(
            String,
            '/teleop_reset',
            self.reset_callback,
            10
        )

    def publish_cmd(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = self.linear_speed
        cmd.twist.angular.z = self.angular_speed

        self.cmd_pub.publish(cmd)

    # Clear stored teleop speeds and publish a stop command.
    def reset_speed(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.publish_cmd()

    def publish_mode(self, mode):
        self.reset_speed()

        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def stop(self):
        self.reset_speed()

    def increase_linear(self):
        self.linear_speed += self.linear_step
        self.linear_speed = min(self.linear_speed, self.max_linear_speed)
        self.publish_cmd()

    def decrease_linear(self):
        self.linear_speed -= self.linear_step
        self.linear_speed = max(self.linear_speed, -self.max_linear_speed)
        self.publish_cmd()

    def turn_left(self):
        self.angular_speed += self.angular_step
        self.angular_speed = min(self.angular_speed, self.max_angular_speed)
        self.publish_cmd()

    def turn_right(self):
        self.angular_speed -= self.angular_step
        self.angular_speed = max(self.angular_speed, -self.max_angular_speed)
        self.publish_cmd()

    def reset_callback(self, msg):
        if msg.data == 'reset':
            self.reset_speed()

# Check for keyboard input without blocking ROS callbacks.
def get_key(timeout=0.1):
    settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setraw(sys.stdin.fileno())

        ready, _, _ = select.select([sys.stdin], [], [], timeout)

        if ready:
            key = sys.stdin.read(1)
        else:
            key = ''

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = SharedTeleopNode()

    print(MSG)

    try:
        while rclpy.ok():
            # Process reset messages while still listening for keyboard input.
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(0.1)

            if key == '':
                continue

            if key == 'w':
                node.increase_linear()
            elif key == 'x':
                node.decrease_linear()
            elif key == 'a':
                node.turn_left()
            elif key == 'd':
                node.turn_right()
            elif key == ' ' or key == 's':
                node.stop()
            elif key == '1':
                node.publish_mode('manual')
            elif key == '2':
                node.publish_mode('assisted')
            elif key == '3':
                node.publish_mode('recovery')
            elif key == '\x03':  # Ctrl-C
                break

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