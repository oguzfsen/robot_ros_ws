#!/usr/bin/env python3

"""
This node listens to topics coming from core_node or autonomous_node. They will send orders to here. 
This node, as a middle label, controls relay_node, hubmotor_node, steer_node and brake_node which are
motion/movement nodes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32, Int32
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
import time

class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node", automatically_declare_parameters_from_overrides=True)

        self.watchdog_frequency = float(_get_param(self, "watchdog_frequency", 10.0))
        self.topic_relay = str(_get_param(self, "topic_relay", "/robot/relay"))
        self.topic_steer = str(_get_param(self, "topic_steer", "/robot/steer"))
        self.topic_hub_motor = str(_get_param(self, "topic_hub_motor", "/robot/hub_motor"))
        self.topic_brake = str(_get_param(self, "topic_brake", "/robot/brake"))
        self.topic_motion = str(_get_param(self, "topic_motion", "/robot/motion"))
        self.topic_average_steering = str(_get_param(self, "topic_average_steering", "/robot/average_steering"))

        self.wheelbase = float(_get_param(self, "wheelbase", 1.6))
        self.wheel_track = float(_get_param(self, "wheel_track", 1.4))
        self.wheel_diameter = float(_get_param(self, "wheel_diameter", 0.4)) # tekerlek çapı
        self.throttle_deadband = int(_get_param(self, "throttle_deadband", 5))

        self.avg_steering = 0
        self.rc_connected = False
        self.rc_armed = False
        self.autonomous_blocked = True
        self.robot_state = RobotState.IDLE
        self.rc_subscription = self.create_subscription(RcInput, str(_get_param(self, "topic_rc_input", "/rc/input")), self.rc_callback, 10)
        self.state_subscription = self.create_subscription(Int32, str(_get_param(self, "topic_state", "/robot/state")), self.state_callback, 10)


        self.ackermann = Ackermann(self.wheelbase, self.wheel_track, self.wheel_diameter)

        self.subsribtion = self.create_subscription(Motion, self.topic_motion, self.motion_callback, 10)
        self.average_steering_subs = self.create_subscription(Float32, self.topic_average_steering, self.avg_steering_callback, 10)
        
        self.relay_publisher = self.create_publisher(RobotRelay, self.topic_relay, 10)
        self.steer_publisher = self.create_publisher(Steer, self.topic_steer, 10)
        self.hubmotor_publisher = self.create_publisher(HubMotor, self.topic_hub_motor, 10)
        self.brake_publisher = self.create_publisher(Brake, self.topic_brake, 10)

        self.command_timeout_sec = float(_get_param(self, "command_timeout_sec", 0.5))
        self.last_command_time = time.monotonic()
        self.watchdog_timer = self.create_timer(1.0 / self.watchdog_frequency, self.watchdog_callback)


    def watchdog_callback(self) -> None:
        """Force steering motors to stop if commands stop arriving."""

        if (time.monotonic() - self.last_command_time) <= self.command_timeout_sec:
            return

        self.stop_everything()


    def rc_callback(self, msg: RcInput) -> None:
        self.rc_connected = bool(msg.connected)
        self.rc_armed = msg.connected and msg.ignition == 1
        if not self.rc_connected or self.rc_armed:
            if not self.autonomous_blocked:
                self.stop_everything()
            self.autonomous_blocked = True

    def state_callback(self, msg: Int32) -> None:
        if msg.data == int(RobotState.MISSION) and self.robot_state != RobotState.MISSION:
            self.autonomous_blocked = not self.rc_connected or self.rc_armed
        self.robot_state = RobotState(msg.data)

    def motion_callback(self, msg: Motion) -> None:
        if msg.autonomous:
            if self.autonomous_blocked or self.robot_state != RobotState.MISSION:
                return
        elif self.robot_state == RobotState.MISSION and not self.autonomous_blocked:
            return
        self.publish_relay(msg.arm, msg.reverse, msg.arm)
        self.publish_steer(msg.steering, msg.arm, msg.stop_steering)
        self.publish_hubmotor(msg.throttle, msg.coast and msg.arm)
        self.last_command_time = time.monotonic()
    

    def stop_everything(self) -> None:
        self.publish_relay(False, False, False)
        self.publish_steer(0, False)
        self.publish_hubmotor(0)


    def publish_relay(self, ignition: bool, reverse: bool, back_energy: bool) -> None:
        relay_msg = RobotRelay()
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.ignition = ignition
        relay_msg.reverse = reverse
        relay_msg.back_energy = back_energy
        relay_msg.brake = False
        relay_msg.alarm_lamp = ignition
        self.relay_publisher.publish(relay_msg)


    def publish_steer(self, steer: int, arm: bool, stop_steering: bool = False) -> None:
        steer_msg = Steer()
        steer_msg.header.stamp = self.get_clock().now().to_msg()
        steer_msg.front_left = steer
        steer_msg.front_right = steer
        steer_msg.rear_left = -steer
        steer_msg.rear_right = -steer
        steer_msg.stop_all = not arm or stop_steering

        self.steer_publisher.publish(steer_msg)


    def publish_hubmotor(self, throttle: int, coast: bool = False) -> None:
        throttle1, throttle2, throttle3, throttle4 = self.ackermann.get_fixed(throttle, self.avg_steering)

        hubmotor_msg = HubMotor()
        hubmotor_msg.header.stamp = self.get_clock().now().to_msg()        
        hubmotor_msg.front_left = throttle1
        hubmotor_msg.front_right = throttle2
        hubmotor_msg.rear_left = throttle3
        hubmotor_msg.rear_right = throttle4

        self.hubmotor_publisher.publish(hubmotor_msg)

        if throttle < self.throttle_deadband and not coast:
            self.brake = True
        else:
            self.brake = False

        brake_msg = Brake()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.brake = self.brake
        self.brake_publisher.publish(brake_msg)

    def avg_steering_callback(self, avg_steering:Float32) -> None:
            self.avg_steering = float(avg_steering.data)


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
