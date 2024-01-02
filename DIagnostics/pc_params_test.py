import psutil
import platform
import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil


class PCDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('pc_diagnostics')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_pc_diagnostics)

    def publish_pc_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Create a DiagnosticStatus for CPU-related information
        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU'
        cpu_status.hardware_id = 'Processor'

        # Get CPU temperature using psutil or other suitable libraries (this is a sample value)
        cpu_temp = 65.2  # in Celsius
        cpu_status.level = DiagnosticStatus.OK if cpu_temp < 80 else DiagnosticStatus.WARN

        cpu_status.message = 'CPU Temperature: {:.2f}°C'.format(cpu_temp)

        # Create KeyValues for additional CPU information (sample values)
        cpu_frequency = psutil.cpu_freq().current  # CPU frequency in MHz
        cpu_load = psutil.cpu_percent(percpu=True)  # CPU load per core

        freq_key_value = KeyValue()
        freq_key_value.key = 'Frequency'
        freq_key_value.value = '{} MHz'.format(cpu_frequency)

        load_key_value = KeyValue()
        load_key_value.key = 'Load per Core'
        load_key_value.value = ', '.join(['Core {}: {:.2f}%'.format(i, load) for i, load in enumerate(cpu_load)])

        # Append KeyValues to the DiagnosticStatus message
        cpu_status.values.append(freq_key_value)
        cpu_status.values.append(load_key_value)

        # Add CPU diagnostic status to the DiagnosticArray message
        msg.status.append(cpu_status)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing PC Diagnostic Information')


def main(args=None):
    rclpy.init(args=args)
    node = PCDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


class DefaultParams:
    print('')
    print('')
    print('---DEFAULT VALUES---')
    print('')
    print('------CPU------')
    print('')

    # Name of CPU
    print(f'Type of CPU: {platform.processor()}')

    # Number of physical cores
    print(f'Number of physical cores: {psutil.cpu_count(logical=False)}')

    # Number of logical cores
    print(f'Number of logical cores: {psutil.cpu_count(logical=True)}')


    # Basic speed
    print(f'Basic speed: {psutil.cpu_freq().max} MHz')

    print('')
    print("------RAM------")
    print('')

    # Default RAM amount
    print(f'Default amount: {round(psutil.virtual_memory().total/1000000000, 2)} GB')


class CurrentParams:
    while True:
        print('')
        print('')
        print('---CURRENT VALUES---')
        print('')
        print('------CPU------')
        print('')

        # Used in percents (average of all cores)
        print(f'Used in percents: {psutil.cpu_percent(interval=1)}%')

        # Basic speed
        print(f'Current speed: {psutil.cpu_freq().current} MHz')

        print('')
        print("------RAM------")
        print('')

        # Currently used
        print(f'Currently used memory: {round(psutil.virtual_memory().used/1000000000, 2)} GB')

        # Currently available memory
        print(f'Currently available memory: {round(psutil.virtual_memory().available/1000000000, 2)} GB')

        # Used in percents
        print(f'Used in percents: {psutil.virtual_memory().percent}%')

        print('')
        print("------NET------")
        print('')

        net_stat = psutil.net_io_counters()
        net_in_1 = net_stat.bytes_recv
        net_out_1 = net_stat.bytes_sent
        time.sleep(5)
        net_stat = psutil.net_io_counters()
        net_in_2 = net_stat.bytes_recv
        net_out_2 = net_stat.bytes_sent

        net_in = round((net_in_2 - net_in_1) / 1024 / 1024, 3)
        net_out = round((net_out_2 - net_out_1) / 1024, 3)

        print(f"Current net-usage in 5 seconds time interval:\nSENT: {net_in} kB/s, RECEIVED: {net_out} kB/s")



