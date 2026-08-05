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

kumanda ignition açılırsa robot_state = MANUAL
    tüm kontrol kullanıcıya geçer.
kumanda ignition kapatılırsa robot_state = IDLE
    kontrolü otonom alabilir.

görev komutları:
    start_mission
    start_spraying
    stop_spraying
    spraying_demo
    Sadece robot_state = IDLE ise çalışır.

    pause_mission
    stop_mission
    Sadece robot_state = MISSION ise çalışır.

    return_home
    robot_state = IDLE veya MISSION ise çalışır.

    emergency
    robot_state = IDLE veya MISSION ise çalışır.

robot_state = EMERGENCY veya MALFUNCTION ise sadece uzaktan kumanda ile kontrol edilebilir. göreve başlamaz. eve dönmez...
EMERGENCY bilinçli durdurmadır. MALFUNCTION ise robottaki bir sorun yüzünden kaynaklanır.

"""

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import (
    RcInput,
    Command,
    Gnss,
    HubMotor,
    RobotRelay,
    Steer,
    Brake,
    Malfunction
)

from robot_core.Buzzer import Buzzer
from robot_core.Led import Led
from robot_core.Tools import RobotState, _get_param
from robot_core.Mission import Mission

class CoreNode(Node):

    def __init__(self) -> None:
        super().__init__("core_node", automatically_declare_parameters_from_overrides=True,)

        self.steer_input_deadband = int(_get_param(self, "steer_input_deadband", 5))
        self.steer_input_max = int(_get_param(self, "steer_input_max", 100))
        self.steer_max = int(_get_param(self, "steer_max", 60))
        self.throttle_deadband = int(_get_param(self, "throttle_deadband", 10))
        self.throttle_input_max = int(_get_param(self, "throttle_input_max", 100))
        self.throttle_max = int(_get_param(self, "throttle_max", 60))

        self.topic_rc_input = str(_get_param(self, "topic_rc_input", "/rc/input"))
        self.topic_gnss = str(_get_param(self, "topic_gnss", "/robot/gnss"))
        self.topic_command = str(_get_param(self, "topic_command", "/user/command"))        
        self.topic_relay = str(_get_param(self, "topic_relay", "/robot/relay"))
        self.topic_steer = str(_get_param(self, "topic_steer", "/robot/steer"))
        self.topic_hub_motor = str(_get_param(self, "topic_hub_motor", "/robot/hub_motor"))
        self.topic_brake = str(_get_param(self, "topic_brake", "/robot/brake"))
        
        self.update_hz = int(_get_param(self, "update_hz", 10))        
        buzzer_gpio = int(_get_param(self, "buzzer_pin", 18))
        led_gpio = int(_get_param(self, "led_pin", 23))

        self.buzzer = Buzzer(buzzer_gpio)
        self.led = Led(led_gpio)

        self.robot_state = RobotState.IDLE
        self.arm_on = False
        self.brake = True
        self.brake_release_time = self.get_clock().now()
        self.rc_steer = 0
        self.rc_throttle = 0
        self.fault_active = False

        self.mission = None

        self.rc_subscription = self.create_subscription(
            RcInput,
            self.topic_rc_input,
            self.rc_callback,
            10,
        )
        self.command_subscription = self.create_subscription(
            Command,
            self.topic_command,
            self.command_callback,
            10
        )
        self.gnss_subscription = self.create_subscription(
            Gnss,
            self.topic_gnss,
            self.gnss_callback,
            10
        )
        self.malfunction_subscription = self.create_subscription(
            Malfunction,
            self.topic_malfunction,
            self.malfunction_callback,
            10
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
        self.brake_publisher = self.create_publisher(
            Brake,
            self.topic_brake,
            10,
        )

        self.create_timer(1.0/self.update_hz, self.update)

        self.get_logger().info("Robot Core up and running.")


    def update(self) -> None:
        if self.robot_state == RobotState.MISSION:
            self.do_mission()

        self.buzzer.update()
        self.led.update(self.robot_state)

        
    def rc_callback(self, msg: RcInput) -> None:
        msg.throttle = self.clamp(0, self.throttle_input_max, msg.throttle)
        msg.steering = self.clamp(-self.steer_input_max, self.steer_input_max, msg.steering)

        if msg.connected and msg.ignition == 1:
            self.robot_state = RobotState.MANUAL
            self.buzzer.arm_on()
            self.arm_on = True
        else:
            self.robot_state = RobotState.IDLE
            self.buzzer.idle()
            self.arm_on = False

        self.publish_relay(msg, enabled=self.arm_on)
        self.publish_steer(msg, enabled=self.arm_on)
        self.publish_hubmotor(msg, enabled=self.arm_on)

    def malfunction_callback(self, msg:Malfunction) -> None:
        if msg.malfunction:
            self.robot_state = RobotState.MALFUNTION
            self.get_logger().error(msg.error)
    

    def command_callback(self, msg:Command) -> None:
        command_type = msg.command_type
        payload = msg.payload

        if command_type == "start_mission":
            mission_id = int(payload)
            self.start_mission(mission_id)
        if command_type == "pause_mission":
            ...
        if command_type == "stop_mission":
            ...
        if command_type == "return_home":
            ...
        if command_type == "start_spraying":
            ...
        if command_type == "stop_spraying":
            ...
        if command_type == "spraying_demo":
            ...
        if command_type == "emergency":
            self.robot_state = RobotState.EMERGENCY


    def start_mission(self, mission_id: int) -> None:
        ...

    def gnss_callback(self, msg:Gnss) -> None:
        ...


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
        self.rc_steer = self.clamp(-self.steer_max, self.steer_max, rc_msg.steering)

        steer_msg = Steer()
        steer_msg.header.stamp = self.get_clock().now().to_msg()

        if enabled:
            steer_msg.front_left_angle = -self.rc_steer
            steer_msg.front_right_angle = -self.rc_steer
            steer_msg.rear_left_angle = self.rc_steer
            steer_msg.rear_right_angle = self.rc_steer
            steer_msg.enable = True
            
        else:
            steer_msg.front_left_angle = 0
            steer_msg.front_right_angle = 0
            steer_msg.rear_left_angle = 0
            steer_msg.rear_right_angle = 0
            steer_msg.enable = False

        self.steer_publisher.publish(steer_msg)


    def publish_hubmotor(self, rc_msg: RcInput, enabled: bool) -> None:
        if rc_msg.throttle > self.throttle_deadband:
            requested_throttle = self.clamp(0, self.throttle_max, rc_msg.throttle)
        else:
            requested_throttle = 0

        release_brake = enabled and requested_throttle > 0
        now = self.get_clock().now()

        # Brake release is commanded immediately, motor power starts 1 second later.
        if release_brake:
            if self.brake:
                self.brake_release_time = now
            self.brake = False

            if (now - self.brake_release_time).nanoseconds >= 1_000_000_000:
                self.rc_throttle = requested_throttle
            else:
                self.rc_throttle = 0
        else:
            self.brake = True
            self.rc_throttle = 0
            self.brake_release_time = now

        hubmotor_msg = HubMotor()
        hubmotor_msg.header.stamp = self.get_clock().now().to_msg()
        hubmotor_msg.enable = enabled
        
        if enabled:
            hubmotor_msg.front_left = self.rc_throttle
            hubmotor_msg.front_right = self.rc_throttle
            hubmotor_msg.rear_left = self.rc_throttle
            hubmotor_msg.rear_right = self.rc_throttle

        else:
            hubmotor_msg.front_left = 0
            hubmotor_msg.front_right = 0
            hubmotor_msg.rear_left = 0
            hubmotor_msg.rear_right = 0


        self.hubmotor_publisher.publish(hubmotor_msg)

        brake_msg = Brake()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.activate = self.brake
        self.brake_publisher.publish(brake_msg)


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
