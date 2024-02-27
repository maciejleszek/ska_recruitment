import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosoutPublisher(Node):

    def __init__(self):
        super().__init__('rosout_publisher')
        self.publisher_ = self.create_publisher(String, '/my_rosout_topic', 10)
        self.timer = self.create_timer(1, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Published message: {}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)

    try:
        rosout_publisher = RosoutPublisher()
        rclpy.spin(rosout_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'rosout_publisher' in locals():
            rosout_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
