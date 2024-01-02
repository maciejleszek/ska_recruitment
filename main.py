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

        cpu_percent_key_value = KeyValue()
        cpu_percent_key_value.key = 'usage_in_percents'
        cpu_percent_key_value.value = str(cpu_percent)

        cpu_freq_key_value = KeyValue()
        cpu_freq_key_value.key = 'frequency'
        cpu_freq_key_value.value = str(cpu_frequency)

        cpu_status.values.append(cpu_percent_key_value)
        cpu_status.values.append(cpu_freq_key_value)

        # RAM
        ram_status = DiagnosticStatus()
        ram_status.name = 'RAM'
        ram_status.hardware_id = f'Default amount: {round(psutil.virtual_memory().total/1000000000, 2)} GB'

        ram_used_memory = round(psutil.virtual_memory().used/1000000000, 2)
        ram_available_memory = round(psutil.virtual_memory().available/1000000000, 2)
        ram_available_memory_percents = psutil.virtual_memory().percent

        ram_status.message = (f'Currently used memory: {ram_used_memory}GB, currently available memory: '
                              f'{ram_available_memory}GB, used in percents: {ram_available_memory_percents}%')

        ram_used_key_value = KeyValue()
        ram_used_key_value.key = 'used_ram'
        ram_used_key_value.value = str(ram_used_memory)

        ram_available_key_value = KeyValue()
        ram_available_key_value.key = 'ram_available_memory'
        ram_available_key_value.value = str(ram_available_memory)

        ram_available_memory_percents_key_value = KeyValue()
        ram_available_memory_percents_key_value.key = 'ram_available_memory_percents'
        ram_available_memory_percents_key_value.value = str(ram_available_memory_percents)

        ram_status.values.append(ram_used_key_value)
        ram_status.values.append(ram_available_key_value)
        ram_status.values.append(ram_available_memory_percents_key_value)

        # NET
        net_status = DiagnosticStatus()
        net_status.name = 'Network'
        net_status.hardware_id = 'Network Interface'

        net_stat = psutil.net_io_counters()
        net_sent_1 = net_stat.bytes_recv
        net_received_1 = net_stat.bytes_sent
        time.sleep(5)  # time interval = 5s
        net_stat = psutil.net_io_counters()
        net_sent_2 = net_stat.bytes_recv
        net_received_2 = net_stat.bytes_sent

        net_sent = round((net_sent_2 - net_sent_1) / 1024 / 1024, 3)
        net_received = round((net_received_2 - net_received_1) / 1024, 3)

        net_status.message = f'Time interval=5s: SENT: {net_sent} kB/s, RECEIVED: {net_received} kB/s'

        net_sent_key_value = KeyValue()
        net_sent_key_value.key = 'net_sent'
        net_sent_key_value.value = str(net_sent)

        net_received_key_value = KeyValue()
        net_received_key_value.key = 'net_received'
        net_received_key_value.value = str(net_received)

        net_status.values.append(net_sent_key_value)
        net_status.values.append(net_received_key_value)

        # Append to status
        msg.status.append(cpu_status)
        msg.status.append(ram_status)
        msg.status.append(net_status)

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
