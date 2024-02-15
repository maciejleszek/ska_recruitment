import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray


class DiagnosticsSubscriber(Node):
    def __init__(self):
        super().__init__('diagnostics_subscriber')

        self.cpu_subscription = self.create_subscription(
            DiagnosticArray, '/telemetry/pc/cpu', self.cpu_callback, 10)
        
        self.memory_subscription = self.create_subscription(
            DiagnosticArray, '/telemetry/pc/memory', self.memory_callback, 10)
        
        self.network_subscription = self.create_subscription(
            DiagnosticArray, '/telemetry/pc/network', self.network_callback, 10)

    def cpu_callback(self, msg):
        self.process_diagnostics('CPU', msg)

    def memory_callback(self, msg):
        self.process_diagnostics('Memory', msg)

    def network_callback(self, msg):
        self.process_diagnostics('Network', msg)

    def process_diagnostics(self, component, msg):
        self.get_logger().info(f'Received diagnostics message for {component}:')
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
