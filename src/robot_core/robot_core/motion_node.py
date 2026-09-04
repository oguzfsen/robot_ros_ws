#!/usr/bin/env python3

"""
This node listens to topics coming from core_node or autonomous_node. They will send orders to here. 
This node, as a middle label, controls relay_node, hubmotor_node, steer_node and brake_node which are
motion/movement nodes
"""

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import (
    RcInput,
    HubMotor,
    RobotRelay,
    Steer,
    Brake,
    Malfunction,
    Emergency,
    Motion
)
from robot_core.Tools import RobotState, _get_param, clamp_int, map_value
from robot_core.Ackermann import Ackermann

class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node", automatically_declare_parameters_from_overrides=True)

        self.topic_relay = str(_get_param(self, "topic_relay", "/robot/relay"))
        self.topic_steer = str(_get_param(self, "topic_steer", "/robot/steer"))
        self.topic_hub_motor = str(_get_param(self, "topic_hub_motor", "/robot/hub_motor"))
        self.topic_brake = str(_get_param(self, "topic_brake", "/robot/brake"))
        self.topic_motion = str(_get_param(self, "topic_motion", "/robot/motion"))
        

        self.wheelbase = float(_get_param(self, "wheelbase", 1.6))
        self.wheel_track = float(_get_param(self, "wheel_track", 1.4))
        self.wheel_diameter = float(_get_param(self, "wheel_diameter", 0.4)) # tekerlek çapı
        self.ackermann = Ackermann(self.wheelbase, self.wheel_track, self.wheel_diameter)

        self.subsribtion = self.create_subscription(Motion, self.topic_motion, self.motion_callback, 1)
        
        self.relay_publisher = self.create_publisher(RobotRelay, self.topic_relay, 10)
        self.steer_publisher = self.create_publisher(Steer, self.topic_steer, 10)
        self.hubmotor_publisher = self.create_publisher(HubMotor, self.topic_hub_motor, 10)
        self.brake_publisher = self.create_publisher(Brake, self.topic_brake, 10)
        

    def motion_callback(self, msg: Motion) -> None:
        self.publish_relay(msg.arm, msg.reverse, msg.arm)
        self.publish_steer(msg.steering)
        self.publish_hubmotor(msg.throttle)
    

    def stop_everything(self) -> None:
        self.publish_relay(False, False, False)
        self.publish_steer(0)
        self.publish_hubmotor(0)


    def publish_relay(self, ignition: bool, reverse: bool, back_energy: bool) -> None:
        relay_msg = RobotRelay()
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.ignition = ignition
        relay_msg.reverse = reverse
        relay_msg.back_energy = back_energy
        relay_msg.brake = False
        relay_msg.alarm_lamp = False
        if self.robot_state == RobotState.MANUAL or self.robot_state == RobotState.MISSION:
            relay_msg.alarm_lamp = True
        
        self.relay_publisher.publish(relay_msg)


    def publish_steer(self, steer: int) -> None:
        steer_msg = Steer()
        steer_msg.header.stamp = self.get_clock().now().to_msg()
        steer_msg.front_left = steer
        steer_msg.front_right = steer
        steer_msg.rear_left = -steer
        steer_msg.rear_right = -steer

        self.steer_publisher.publish(steer_msg)


    def publish_hubmotor(self, throttle: int) -> None:

        throttle1, throttle2, throttle3, throttle4 = self.ackermann.get_fixed(throttle, self.avg_steering)

        hubmotor_msg = HubMotor()
        hubmotor_msg.header.stamp = self.get_clock().now().to_msg()        
        hubmotor_msg.front_left = throttle1
        hubmotor_msg.front_right = throttle2
        hubmotor_msg.rear_left = throttle3
        hubmotor_msg.rear_right = throttle4

        self.hubmotor_publisher.publish(hubmotor_msg)

        if throttle < self.throttle_deadband:
            self.brake = True
        else:
            self.brake = False

        brake_msg = Brake()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.brake = self.brake
        self.brake_publisher.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)

    node = MotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()