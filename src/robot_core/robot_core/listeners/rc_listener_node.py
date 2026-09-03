import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RcInput
from robot_core.Tools import _get_param

class RCListenerNode(Node):
    def __init__(self) -> None:
        super().__init__("rc_listener_node", automatically_declare_parameters_from_overrides=True)
        
        self.topic_rc_input = str(_get_param(self, "topic_rc_input", "/rc/input"))

        self.raw_line = ""

        self.create_subscription(RcInput, self.topic_rc_input, self.rc_listen_callback, 10)
        self.create_timer(1.0, self.timer_callback)

    def rc_listen_callback(self, msg: RcInput) -> None:
        self.raw_line = msg.raw_line
    
    def timer_callback(self) -> None:
        self.get_logger().info(self.raw_line)

    def destroy_node(self) -> bool:
        return super().destroy_node()
        


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RCListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()