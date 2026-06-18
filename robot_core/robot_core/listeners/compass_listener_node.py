import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Compass


class CompassListenerNode(Node):
    def __init__(self):
        super().__init__("compass_listener_node", automatically_declare_parameters_from_overrides=True)

        self.subscription = self.create_subscription(Compass, "/robot/compass", self.subs_callback, 10)
        self.create_timer(1, self.timer_callback)

        self.data = None


    def subs_callback(self, msg : Compass):
        self.data = msg.heading_deg
        
    def timer_callback(self):
        self.get_logger().info(str(self.data))

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None):
    """
    Start the steering encoder node.
    """
    rclpy.init(args=args)

    node = CompassListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()