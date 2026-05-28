import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import String


class StatusMonitorNode(Node):

    def __init__(self):
        super().__init__('status_monitor')

        print()
        print('Shared-Control Status Monitor')
        print('-----------------------------')

        self.create_subscription(
            String,
            '/shared_control_status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        print(msg.data)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = StatusMonitorNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()