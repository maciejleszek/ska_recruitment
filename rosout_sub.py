import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

class LogListener(Node):
    def __init__(self):
        super().__init__('log_listener', enable_logger_service=True)
        self.subscription = self.create_subscription(Log, '/rosout', self.callback, 10)

    def callback(self, msg):
        print(msg)
        print("\n")

def main():
    rclpy.init()
    node = LogListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
