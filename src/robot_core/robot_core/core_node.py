import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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

        self.throttle_max = float(_get_param(self, "throttle_deadzone", 80.0))
        self.throttle_deadzone = float(_get_param(self, "throttle_deadzone", 5.0))
        self.update_hz = int(_get_param(self, "update_hz"), 10)
        buzzer_gpio_pin = int(_get_param(self, "buzzer_pin", 18))
        led_gpio_pin = int(_get_param(self, "led_pin", 23)),

        self.buzzer = Buzzer(buzzer_gpio_pin)
        self.led = Led(led_gpio_pin)

        self.robot_state = RobotState.IDLE

        self.relay_msg = RobotRelay()
        self.steer_msg = Steer()
        self.hubmotor_msg = HubMotor()

        self.rc_steer = 0
        self.last_armed = False
        self.last_moving = False
        self.fault_active = False

        self.rc_subscription = self.create_subscription(
            RcInput,
            "/rc/input",
            self.rc_callback,
            10,
        )
        self.relay_publisher = self.create_publisher(
            RobotRelay,
            "/robot/relay",
            10,
        )
        self.steer_publisher = self.create_publisher(
            Steer,
            "/robot/steer",
            10,
        )
        self.hubmotor_publisher = self.create_publisher(
            HubMotor,
            "/robot/hub_motor",
            10,
        )

        self.create_timer(self.update_hz, self.update)

        self.get_logger().info("Robot Core up and running.")


    def update(self) -> None:
        self.buzzer.update()
        self.led.update(self.robot_state)
        self.publish_diag()

    def publish_diag(self) -> None:
        ...


        
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

    def handle_moving_buzzer(self, moving: bool) -> None:
        if moving == self.last_moving:
            return

        self.last_moving = moving
        self.buzzer.set_moving(moving)

    def publish_relay(self, rc_msg: RcInput, enabled: bool) -> None:
        self.relay_msg.header.stamp = self.get_clock().now().to_msg()
        if enabled:
            self.relay_msg.ignition = rc_msg.ignition == 1
            self.relay_msg.reverse = rc_msg.reverse == 2
            self.relay_msg.brake = False
            self.relay_msg.back_energy = False
            self.relay_msg.alarm_lamp = self.relay_msg.ignition
        else:
            self.relay_msg.ignition = False
            self.relay_msg.reverse = False
            self.relay_msg.brake = False
            self.relay_msg.back_energy = False
            self.relay_msg.alarm_lamp = False
        self.relay_publisher.publish(self.relay_msg)

    def publish_steer(self, rc_msg: RcInput, enabled: bool) -> None:
        """
        Publish a simple four-wheel steering target derived from RC input.
        """
        self.rc_steer = rc_msg.steering
        self.rc_steer = int(self.clamp(-100, 100, rc_msg.steering))

        if enabled:
            self.steer_msg.header.stamp = self.get_clock().now().to_msg()
            self.steer_msg.front_left_angle = self.rc_steer
            self.steer_msg.front_right_angle = self.rc_steer
            self.steer_msg.rear_left_angle = -self.rc_steer
            self.steer_msg.rear_right_angle = -self.rc_steer
            self.steer_msg.enable = True
            self.steer_publisher.publish(self.steer_msg)
        else:
            self.steer_msg.header.stamp = self.get_clock().now().to_msg()
            self.steer_msg.front_left_angle = 0
            self.steer_msg.front_right_angle = 0
            self.steer_msg.rear_left_angle = 0
            self.steer_msg.rear_right_angle = 0
            self.steer_msg.enable = False
            self.steer_publisher.publish(self.steer_msg)

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

        self.hubmotor_msg.header.stamp = self.get_clock().now().to_msg()
        self.hubmotor_msg.front_left = wheel_power
        self.hubmotor_msg.front_right = wheel_power
        self.hubmotor_msg.rear_left = wheel_power
        self.hubmotor_msg.rear_right = wheel_power
        self.hubmotor_msg.enable = enabled

        self.hubmotor_publisher.publish(self.hubmotor_msg)
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
