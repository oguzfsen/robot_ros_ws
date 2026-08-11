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
    Malfunction,
    Emergency,
)
from robot_interfaces.srv import MissionGet
from std_msgs.msg import Int32MultiArray, Float32
from robot_core.Buzzer import Buzzer
from robot_core.Tools import RobotState, _get_param
from robot_core.Mission import Mission
from robot_core.Ackermann import Ackermann

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
        self.topic_malfunction = str(_get_param(self, "topic_malfunction", "/robot/malfunction"))
        self.topic_emergency = str(_get_param(self, "topic_emergency", "/robot/emergency"))
        
        self.update_hz = int(_get_param(self, "update_hz", 10))        
        buzzer_gpio = int(_get_param(self, "buzzer_pin", 18))
        led_gpio = int(_get_param(self, "led_pin", 23))

        self.wheelbase = float(_get_param(self, "wheelbase", 1.6))
        self.wheel_track = float(_get_param(self, "wheel_track", 1.4))
        self.wheel_diameter = float(_get_param(self, "wheel_diameter", 0.4)) # tekerlek çapı
        self.ackermann = Ackermann(self.wheelbase, self.wheel_track, self.wheel_diameter)


        self.buzzer = Buzzer(buzzer_gpio)
        self.mission = None

        self.robot_state = RobotState.IDLE
        self.rc_ignition = False
        self.rc_reverse = False
        self.rc_steering = 0
        self.rc_throttle = 0
        self.brake = False
        self.brake_release_time = self.get_clock().now()
        self.back_energy = False
        self.avg_steering = 0.0

        self.accuracy_m = 999.0
        self.ground_speed_m_s = 0.0
        self.latitude = 0.0
        self.longitude = 0.0

        self.rc_subscription = self.create_subscription(RcInput, self.topic_rc_input, self.rc_callback, 10)
        self.command_subscription = self.create_subscription(Command, self.topic_command, self.command_callback, 10)
        self.gnss_subscription = self.create_subscription(Gnss, self.topic_gnss, self.gnss_callback, 10)
        self.malfunction_subscription = self.create_subscription(Malfunction, self.topic_malfunction, self.malfunction_callback, 10)
        self.emergency_subscription = self.create_subscription(Emergency, self.topic_emergency, self.emergency_callback, 10)
        self.average_steering_subscription = self.create_subscription(Float32, "/robot/average_steering", self.avg_steering_callback, 10)
        

        self.relay_publisher = self.create_publisher(RobotRelay, self.topic_relay, 10)
        self.steer_publisher = self.create_publisher(Steer, self.topic_steer, 10)
        self.hubmotor_publisher = self.create_publisher(HubMotor, self.topic_hub_motor, 10)
        self.brake_publisher = self.create_publisher(Brake, self.topic_brake, 10)

        self.mission_puller = self.create_client(MissionGet,"/database/mission_get")

        self.create_timer(1.0/self.update_hz, self.update)

        self.get_logger().info("Robot Core up and running.")



    def update(self) -> None:
        if self.robot_state == RobotState.MISSION:
            self.do_mission()

        self.buzzer.update()
        

        
    def rc_callback(self, msg: RcInput) -> None:

        if msg.connected:
            self.rc_ignition = msg.ignition == 1
            self.rc_reverse = msg.reverse == 2

            if self.rc_ignition:
                self.robot_state = RobotState.MANUAL
                self.rc_throttle = self.clamp(0, self.throttle_input_max, msg.throttle)
                self.rc_steering = self.clamp(-self.steer_input_max, self.steer_input_max, msg.steering)
            else:
                if self.robot_state != RobotState.MISSION:
                    self.robot_state = RobotState.IDLE
                self.rc_reverse = False
                self.rc_throttle = 0
                self.rc_steering = 0

            self.publish_relay(self.rc_ignition, self.rc_reverse, self.back_energy)
            self.publish_steer(self.rc_steering)
            self.publish_hubmotor(self.rc_throttle)
                    
        else:
            ... # do nothing


    def malfunction_callback(self, msg:Malfunction) -> None:
        if msg.malfunction:
            self.robot_state = RobotState.MALFUNTION
            self.get_logger().error(msg.error)

    def emergency_callback(self, msg:Emergency) -> None:
        if msg.emergency:
            self.robot_state = RobotState.EMERGENCY
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

        if self.robot_state != RobotState.IDLE:
            self.get_logger().error("Robot beklemede değilken görev başlatılamaz!")
            return
        
        request = MissionGet.Request()
        request.mission_id = mission_id
        future = self.mission_puller.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec = 1.0,
        )

        if not future.done():
            raise RuntimeError("MissionGet service timeout")

        response = future.result()
        if response is None:
            raise RuntimeError("MissionGet service returned no response")
        if not response.success:
            raise RuntimeError(f"MissionGet service failed: {response.message}")

        self.mission = Mission(future.raw_json, 0.5, future.last_waypoint)
        if self.mission.has_next_waypoint:
            self.robot_state = RobotState.MISSION
        else:
            self.get_logger().error("Görevde sonraki nokta yok!")


    def gnss_callback(self, msg:Gnss) -> None:
        self.accuracy_m = msg.accuracy_m
        self.ground_speed_m_s = msg.ground_speed_m_s
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        

    def avg_steering_callback(self, avg_steering:Float32) -> None:
        self.avg_steering = float(avg_steering.data)


    def publish_relay(self, ignition: bool, reverse: bool, back_energy: bool) -> None:
        relay_msg = RobotRelay()
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.ignition = ignition
        relay_msg.reverse = reverse
        relay_msg.back_energy = back_energy
        relay_msg.brake = False
        relay_msg.alarm_lamp = False
        
        self.relay_publisher.publish(relay_msg)


    def publish_steer(self, steer: int) -> None:
        steer_msg = Steer()
        steer_msg.header.stamp = self.get_clock().now().to_msg()
        steer_msg.front_left = -steer
        steer_msg.front_right = -steer
        steer_msg.rear_left = steer
        steer_msg.rear_right = steer
        steer_msg.enable = self.rc_ignition         

        self.steer_publisher.publish(steer_msg)


    def publish_hubmotor(self, throttle: int) -> None:

        self.get_logger().info(str(self.avg_steering))
        throttle1, throttle2, throttle3, throttle4 = self.ackermann.get_fixed(throttle, self.avg_steering)

        hubmotor_msg = HubMotor()
        hubmotor_msg.header.stamp = self.get_clock().now().to_msg()        
        hubmotor_msg.front_left = throttle1
        hubmotor_msg.front_right = throttle2
        hubmotor_msg.rear_left = throttle3
        hubmotor_msg.rear_right = throttle4
        hubmotor_msg.enable = self.rc_ignition

        self.hubmotor_publisher.publish(hubmotor_msg)

        if throttle < self.throttle_deadband:
            self.brake = True
        else:
            self.brake = False

        brake_msg = Brake()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.brake = self.brake
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
