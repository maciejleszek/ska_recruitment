import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray


class DiagnosticsSubscriber(Node):
    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.subscription = self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)

    def diagnostics_callback(self, msg):
        self.get_logger().info('Received diagnostics message:')
        for status in msg.status:
            self.get_logger().info(f'  Name: {status.name}, Hardware ID: {status.hardware_id}')
            self.get_logger().info(f'  Message: {status.message}')
            self.get_logger().info('  Values:')
            for kv in status.values:
                self.get_logger().info(f'    {kv.key}: {kv.value}')


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
