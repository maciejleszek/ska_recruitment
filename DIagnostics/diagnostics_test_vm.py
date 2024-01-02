import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil
import platform
import time

class PcDiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('pc_diagnostic_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        battery_status = DiagnosticStatus()
        battery_status.name = 'PC'
        battery_status.hardware_id = 'Battery Sensor'

        battery_level = random.uniform(0, 100)  # Simulate battery level (0-100)
        battery_status.level = DiagnosticStatus.OK if battery_level > 20 else DiagnosticStatus.WARN

        battery_status.message = 'Battery level: {:.2f}%'.format(battery_level)

        key_value = KeyValue()
        key_value.key = 'Voltage'
        key_value.value = '{:.2f} V'.format(random.uniform(12, 14))  # Simulate voltage
        battery_status.values.append(key_value)

        msg.status.append(battery_status)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing PC Diagnostic Information')

def main(args=None):
    rclpy.init(args=args)
    node = PcDiagnosticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
