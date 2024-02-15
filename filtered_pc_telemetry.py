import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from datetime import datetime

class TelemetryFilterNode(Node):
    def __init__(self):
        super().__init__('telemetry_filter')
        self.subscription_ram = self.create_subscription(DiagnosticArray, '/telemetry/pc/ram', self.ram_callback, 10)
        self.subscription_memory = self.create_subscription(DiagnosticArray, '/telemetry/pc/memory', self.memory_callback, 10)
        self.subscription_cpu = self.create_subscription(DiagnosticArray, '/telemetry/pc/cpu', self.cpu_callback, 10)
        self.subscription_network = self.create_subscription(DiagnosticArray, '/telemetry/pc/network', self.network_callback, 10)

    def check_and_log_value(self, name, value, threshold):
        if value > threshold:
            error_msg = f'Error in {name}: Value ({value}) exceeds threshold ({threshold}) at {self.get_current_time()}'
            self.get_logger().error(error_msg)

    def get_current_time(self):
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    def process_diagnostic_status(self, status):
        for kv in status.values:
            if kv.key == 'net_sent' or kv.key == 'net_received':
                self.check_and_log_value(f'{status.name} - {kv.key}', float(kv.value), 10000)  # Threshold for network values (10000 kB/s)
            elif status.name == 'CPU' and kv.key == 'usage_in_percents':
                self.check_and_log_value(f'{status.name} - {kv.key}', float(kv.value), 80)  # Threshold for CPU usage (80%)
            elif status.name == 'RAM' and kv.key == 'used_ram':
                self.check_and_log_value(f'{status.name} - {kv.key}', float(kv.value), 80)  # Threshold for RAM usage (80%)

    def ram_callback(self, msg):
        for status in msg.status:
            self.process_diagnostic_status(status)

    def memory_callback(self, msg):
        for status in msg.status:
            self.process_diagnostic_status(status)

    def cpu_callback(self, msg):
        for status in msg.status:
            self.process_diagnostic_status(status)

    def network_callback(self, msg):
        for status in msg.status:
            self.process_diagnostic_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
