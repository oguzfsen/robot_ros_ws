#!/usr/bin/env python3

"""
Brake node.

...
"""

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Brake
from robot_core.Tools import _get_param


class BrakeNode(Node):
    """
    
    """

    def __init__(self):
        super().__init__("brake_node", automatically_declare_parameters_from_overrides=True)
        

        self.frequency = float(_get_param(self, "frequency", 10.0))

        ...
        
        self.get_logger().info("brake_node up and running")

    
 


def main(args=None):
    rclpy.init(args=args)

    node = BrakeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
