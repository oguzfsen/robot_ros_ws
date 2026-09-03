import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Compass
from robot_core.Tools import _get_param


class CompassListenerNode(Node):
    def __init__(self):
        super().__init__("compass_listener_node", automatically_declare_parameters_from_overrides=True)

        self.topic_compass = str(_get_param(self, "topic_compass", "/robot/compass"))

        self.subscription = self.create_subscription(Compass, self.topic_compass, self.subs_callback, 10)
        self.create_timer(1, self.timer_callback)

        self.heading_deg = None


    def subs_callback(self, msg : Compass):
        self.heading_deg = msg.heading_deg
        
    def timer_callback(self):
        self.get_logger().info(str(self.heading_deg))

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None):
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