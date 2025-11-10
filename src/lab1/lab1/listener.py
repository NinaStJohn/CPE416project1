import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from std_msgs.msg import Int32

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subsription object
        self.subscription = self.create_subscription(
            Int32,
            'numbers',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        squ = msg.data * msg.data
        self.get_logger().info('Result: "%i"' % squ)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
