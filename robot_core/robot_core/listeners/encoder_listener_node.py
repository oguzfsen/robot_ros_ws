import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class EncoderListenerNode(Node):
    def __init__(self):
        super().__init__("encoder_listener_node")

        self.subscription = self.create_subscription(Int32MultiArray, "/robot/steering_encoders", self.subs_callback, 10)
        self.create_timer(1, self.timer_callback)

        self.data = []


    def subs_callback(self, msg : Int32MultiArray):
        self.data = msg.data
        
    def timer_callback(self):
        self.get_logger().info(str(self.data))

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None):
    """
    Start the steering encoder node.
    """
    rclpy.init(args=args)

    node = EncoderListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()