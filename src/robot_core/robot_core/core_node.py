#!/usr/bin/env python3
"""
core_node robot ros sisteminin yönetim merkezidir.
Şu Topicleri dinleyerek bilgi toplar:
/rc/input
/robot/gnss
/user/command
Şu Topiclerle robotu yönetir:
/robot/relay
/robot/steer
/robot/hub_motor
"""


import rclpy
from rclpy.node import Node
from robot_interfaces.msg import (
    HubMotor,
    RcInput,
    RobotRelay,
    Steer,
)

from robot_core.Buzzer import Buzzer
from robot_core.Led import Led
from robot_core.Tools import RobotState, _get_param

class CoreNode(Node):

    def __init__(self) -> None:
        super().__init__("core_node", automatically_declare_parameters_from_overrides=True,)

        self.steering_center = float(_get_param("steering_center", 0.0))
        self.steering_input_max = float(_get_param("steering_input_max", 100.0))
        self.steering_deadband = float(_get_param("steering_deadband", 5.0))
        self.max_steer_angle = float(_get_param("max_steer_angle", 70.0))
        self.throttle_center = float(_get_param(self, "throttle_center", 0.0))
        self.throttle_input_max = float(_get_param(self, "throttle_input_max", 100.0))
        self.throttle_deadband = float(_get_param(self, "throttle_deadband", 5.0))
        self.update_hz = int(_get_param(self, "update_hz", 10))        
        self.topic_rc_input = str(_get_param("topic_rc_input", "/rc/input"))
        self.topic_gnss = str(_get_param("topic_gnss", "/robot/gnss"))
        self.topic_command = str(_get_param("topic_command", "/user/command"))        
        self.topic_relay = str(_get_param("topic_relay", "/robot/relay"))
        self.topic_steer = str(_get_param("topic_steer", "/robot/steer"))
        self.topic_hub_motor = str(_get_param("topic_hub_motor", "/robot/hub_motor"))

        buzzer_gpio = int(_get_param(self, "buzzer_pin", 18))
        led_gpio = int(_get_param(self, "led_pin", 23))
        self.buzzer = Buzzer(buzzer_gpio)
        self.led = Led(led_gpio)

        self.robot_state = RobotState.IDLE
        self.rc_steer = 0
        self.last_armed = False
        self.fault_active = False

        self.rc_subscription = self.create_subscription(
            RcInput,
            self.topic_rc_input,
            self.rc_callback,
            10,
        )
        self.command_subscription = self.create_subscription(
            ...
        )
        self.relay_publisher = self.create_publisher(
            RobotRelay,
            self.topic_relay,
            10,
        )
        self.steer_publisher = self.create_publisher(
            Steer,
            self.topic_steer,
            10,
        )
        self.hubmotor_publisher = self.create_publisher(
            HubMotor,
            self.topic_hub_motor,
            10,
        )

        self.create_timer(self.update_hz, self.update)

        self.get_logger().info("Robot Core up and running.")


    def update(self) -> None:
        self.buzzer.update()
        self.led.update(self.robot_state)

        
    def rc_callback(self, msg: RcInput) -> None:
        self.arm_on = msg.connected and msg.ignition == 1

        self.publish_relay(msg, enabled=False)
        self.publish_steer(msg, enabled=False)
        self.publish_hubmotor(msg, enabled=False)

        self.buzzer.update()
        self.led.update(self.robot_state)
        

    def set_current_mode(self, mode: str) -> None:
        next_mode = str(mode).strip().lower()
        if next_mode == self.current_mode:
            return

        self.current_mode = next_mode
        self.update_status_led()


    def update_status_led(self) -> None:
        if self.fault_active or self.current_mode == self.MODE_FAILSAFE:
            self.led.set_mode(Led.MODE_FAILSAFE)
            return

        if self.current_mode == self.MODE_MISSION:
            self.led.set_mode(Led.MODE_MISSION)
            return

        if self.current_mode == self.MODE_MANUAL:
            self.led.set_mode(Led.MODE_MANUAL)
            return

        if self.current_mode == self.MODE_CHARGE:
            self.led.set_mode(Led.MODE_CHARGE)
            return

        self.led.set_mode(Led.MODE_IDLE)

    def handle_arm_buzzer(self, armed: bool) -> None:
        if armed == self.last_armed:
            return

        self.last_armed = armed
        self.buzzer.set_armed(armed)


    def publish_relay(self, rc_msg: RcInput, enabled: bool) -> None:
        relay_msg = RobotRelay()
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        if enabled:
            relay_msg.ignition = rc_msg.ignition == 1
            relay_msg.reverse = rc_msg.reverse == 2
            relay_msg.brake = False
            relay_msg.back_energy = False
            relay_msg.alarm_lamp = relay_msg.ignition
        else:
            relay_msg.ignition = False
            relay_msg.reverse = False
            relay_msg.brake = False
            relay_msg.back_energy = False
            relay_msg.alarm_lamp = False
        self.relay_publisher.publish(relay_msg)

    def publish_steer(self, rc_msg: RcInput, enabled: bool) -> None:
        self.rc_steer = rc_msg.steering
        self.rc_steer = int(self.clamp(-100, 100, rc_msg.steering))

        steer_msg = Steer()

        if enabled:
            steer_msg.header.stamp = self.get_clock().now().to_msg()
            steer_msg.front_left_angle = self.rc_steer
            steer_msg.front_right_angle = self.rc_steer
            steer_msg.rear_left_angle = -self.rc_steer
            steer_msg.rear_right_angle = -self.rc_steer
            steer_msg.enable = True
            
        else:
            steer_msg.header.stamp = self.get_clock().now().to_msg()
            steer_msg.front_left_angle = 0
            steer_msg.front_right_angle = 0
            steer_msg.rear_left_angle = 0
            steer_msg.rear_right_angle = 0
            steer_msg.enable = False

        self.steer_publisher.publish(steer_msg)


    def publish_hubmotor(self, rc_msg: RcInput, enabled: bool) -> None:
        """
        Publish a simple four-wheel hub motor power command from RC throttle.

        The hub motor message carries power magnitude.
        Direction is handled elsewhere.
        """
        throttle_command = 0
        if rc_msg.throttle > self.throttle_deadzone:
            throttle_command = rc_msg.throttle

        if enabled:
            throttle_command = self.rc_throttle_to_command(
                float(rc_msg.throttle)
            )

        wheel_power = min(max(0, throttle_command), 100)

        hubmotor_msg = HubMotor()
        hubmotor_msg.header.stamp = self.get_clock().now().to_msg()
        hubmotor_msg.front_left = wheel_power
        hubmotor_msg.front_right = wheel_power
        hubmotor_msg.rear_left = wheel_power
        hubmotor_msg.rear_right = wheel_power
        hubmotor_msg.enable = enabled

        self.hubmotor_publisher.publish(hubmotor_msg)
        self.handle_moving_buzzer(enabled and wheel_power > 0)

    def rc_throttle_to_command(self, throttle_value: float) -> int:
        """
        Convert RC throttle input to a signed percentage command.
        """
        centered_value = throttle_value - self.throttle_center

        if abs(centered_value) <= self.throttle_deadband:
            return 0

        if self.throttle_input_max <= 0.0:
            return 0

        limited = max(
            -self.throttle_input_max,
            min(self.throttle_input_max, centered_value),
        )
        return int(round((limited / self.throttle_input_max) * 100.0))
    

    def clamp(self, min_val, max_val, value) -> int:
        """Clamp a value as int"""
        return max(min_val, min(max_val, value))

    def destroy_node(self) -> bool:
        try:
            self.buzzer.shutdown()
        except Exception:
            pass

        try:
            self.led.shutdown()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoreNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
