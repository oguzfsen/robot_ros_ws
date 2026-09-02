#!/usr/bin/env python3

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
    Compass,
)
from robot_interfaces.srv import MissionGet
from std_msgs.msg import Int32MultiArray, Float32
from robot_core.Buzzer import Buzzer
from robot_core.Tools import RobotState, _get_param, clamp_int, map_value
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
        self.topic_average_steering = str(_get_param(self, "topic_average_steering", "/robot/average_steering"))
        self.topic_mission_get = str(_get_param(self, "topic_mission_get", "/database/mission_get"))
        self.topic_compass = str(_get_param(self, "topic_compass", "/robot/compass"))
        
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
        self.back_energy = False
        self.avg_steering = 0.0

        self.gnss: Gnss | None = None
        self.compass: Compass | None = None

        self.compass_heading = 0.0

        self.rc_subscription = self.create_subscription(RcInput, self.topic_rc_input, self.rc_callback, 10)
        self.command_subscription = self.create_subscription(Command, self.topic_command, self.command_callback, 10)
        self.gnss_subscription = self.create_subscription(Gnss, self.topic_gnss, self.gnss_callback, 10)
        self.malfunction_subscription = self.create_subscription(Malfunction, self.topic_malfunction, self.malfunction_callback, 10)
        self.emergency_subscription = self.create_subscription(Emergency, self.topic_emergency, self.emergency_callback, 10)
        self.average_steering_subscription = self.create_subscription(Float32, self.topic_average_steering, self.avg_steering_callback, 10)
        self.compass_subscription = self.create_subscription(Compass, self.topic_compass, self.compass_callback, 10)

        self.relay_publisher = self.create_publisher(RobotRelay, self.topic_relay, 10)
        self.steer_publisher = self.create_publisher(Steer, self.topic_steer, 10)
        self.hubmotor_publisher = self.create_publisher(HubMotor, self.topic_hub_motor, 10)
        self.brake_publisher = self.create_publisher(Brake, self.topic_brake, 10)

        self.mission_puller = self.create_client(MissionGet, self.topic_mission_get)

        self.create_timer(1.0/self.update_hz, self.update)

    def update(self) -> None:
        if self.robot_state == RobotState.MALFUNTION:
            self.stop_everything()
        elif self.robot_state == RobotState.EMERGENCY:
            self.stop_everything()
        elif self.robot_state == RobotState.MANUAL:
            self.manual_control()
        elif self.robot_state == RobotState.MISSION:
            self.do_mission()
        
        
        self.buzzer.update()

    def manual_control(self):
        self.publish_relay(self.rc_ignition, self.rc_reverse, self.back_energy)
        self.publish_steer(self.rc_steering)
        self.publish_hubmotor(self.rc_throttle)


    def rc_callback(self, msg: RcInput) -> None:

        if not msg.connected:
            if self.robot_state == RobotState.MANUAL or self.robot_state == RobotState.MISSION:
                self.robot_state == RobotState.IDLE
                self.stop_everything()
            return

        # rc açık, tüm değerleri çek
        self.rc_ignition = msg.ignition == 1
        self.rc_reverse = msg.reverse == 2

        if msg.throttle < self.throttle_deadband:
            self.rc_throttle = 0
        else:
            self.rc_throttle = map_value(msg.throttle, 0, 100, 0, self.throttle_input_max)

        if msg.steering < self.steer_input_deadband and msg.steering > -self.steer_input_deadband:
            self.rc_steering = 0
        else:
            self.rc_steering = map_value(msg.steering, -100, 100, -self.steer_input_max, self.steer_input_max)
        
        # rc arm oldu, KOŞULSUZ manual moda geç
        if self.rc_ignition:
            self.robot_state = RobotState.MANUAL
            return

        # manual moddaydın ama arm kapandı        
        if self.robot_state == RobotState.MANUAL:
            self.robot_state = RobotState.IDLE
            self.stop_everything()
            return


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
        future.add_done_callback(self._mission_received)


    def _mission_received(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Görev alınırken servis hatası: {exc}")
            return

        if response is None:
            self.get_logger().error("MissionGet servisi cevap döndürmedi!")
            return

        if not response.raw_json:
            self.get_logger().error(f"Görev verisi boş! mission_id={response.mission_id}")
            return

        try:
            self.mission = Mission(
                response.raw_json,
                0.5,
                # response.last_waypoint, # Baştan başlasın
            )
        except Exception as exc:
            self.get_logger().error(f"Mission nesnesi oluşturulamadı: {exc}")
            return

        if not self.mission.has_next_waypoint:
            self.get_logger().error(
                f"Görevde sonraki waypoint yok! "
                f"mission_id={response.mission_id}, "
            )
            self.mission = None
            return

        self.robot_state = RobotState.MISSION

        self.get_logger().info(
            f"Görev başlatıldı. "
            f"mission_id={response.mission_id}, "
        )

    def gnss_callback(self, msg:Gnss) -> None:
        self.gnss = msg
        

    def compass_callback(self, msg: Compass) -> None:
        self.compass = msg


    def avg_steering_callback(self, avg_steering:Float32) -> None:
        self.avg_steering = float(avg_steering.data)


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
        steer_msg.front_left = -steer
        steer_msg.front_right = -steer
        steer_msg.rear_left = steer
        steer_msg.rear_right = steer

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
