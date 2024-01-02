import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil
import platform
import time

class PcDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('pc_diagnostics')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_pc_diagnostics)

    def publish_pc_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # CPU
        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU'
        cpu_status.hardware_id = f'{platform.processor()}'

        cpu_percent = psutil.cpu_percent()
        cpu_frequency = psutil.cpu_freq().current
        cpu_status.message = f'Usage in percents: {cpu_percent}%, Current frequency: {cpu_frequency}MHz'

        # RAM
        ram_status = DiagnosticStatus()
        ram_status.name = 'RAM'
        ram_status.hardware_id = f'Default amount: {round(psutil.virtual_memory().total/1000000000, 2)} GB'

        ram_used_memory = round(psutil.virtual_memory().used/1000000000, 2)
        ram_available_memory = round(psutil.virtual_memory().available/1000000000, 2)
        ram_available_memory_percents = psutil.virtual_memory().percent

        ram_status.message = f'Currently used memory: {ram_used_memory}GB, currently available memory: {ram_available_memory}GB, used in percents: {ram_available_memory_percents}%'

        # NET
        network_status = DiagnosticStatus()
        network_status.name = 'Network'
        network_status.hardware_id = 'Network Interface'

        net_stat = psutil.net_io_counters()
        net_in_1 = net_stat.bytes_recv
        net_out_1 = net_stat.bytes_sent
        time.sleep(5) # time interval = 5s
        net_stat = psutil.net_io_counters()
        net_in_2 = net_stat.bytes_recv
        net_out_2 = net_stat.bytes_sent

        net_in = round(
            (net_in_2 - net_in_1) / 1024 / 1024,
            3)
        net_out = round(
            (net_out_2 - net_out_1) / 1024, 3)

        network_status.message = f'Time interval=5s: SENT: {net_in} kB/s, RECEIVED: {net_out} kB/s'

        #Append to status
        msg.status.append(cpu_status)
        msg.status.append(ram_status)
        msg.status.append(network_status)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing PC Diagnostic Information')

def main(args=None):
    rclpy.init(args=args)
    node = PcDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
