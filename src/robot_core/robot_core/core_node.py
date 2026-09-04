#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import (
    RcInput,
    Command,
    Gnss,
    Malfunction,
    Emergency,
    Compass,
    Motion
)
from robot_interfaces.srv import MissionGet
from std_msgs.msg import Int32MultiArray, Float32
from robot_core.Buzzer import Buzzer
from robot_core.Tools import RobotState, _get_param, clamp_int, map_value
from robot_core.Mission import Mission


class CoreNode(Node):

    def __init__(self) -> None:
        super().__init__("core_node", automatically_declare_parameters_from_overrides=True,)

        self.throttle_deadband = int(_get_param(self, "throttle_deadband", 5))
        self.throttle_input_max = int(_get_param(self, "throttle_input_max", 100))
        self.throttle_max = int(_get_param(self, "throttle_max", 60))
        self.steer_input_deadband = int(_get_param(self, "steer_input_deadband", 5))
        self.steer_input_max = int(_get_param(self, "steer_input_max", 100)) # (-100 ile +100 arası)
        self.steer_max = int(_get_param(self, "steer_max", 60))

        self.topic_rc_input = str(_get_param(self, "topic_rc_input", "/rc/input"))
        self.topic_gnss = str(_get_param(self, "topic_gnss", "/robot/gnss"))
        self.topic_command = str(_get_param(self, "topic_command", "/user/command"))        
        self.topic_malfunction = str(_get_param(self, "topic_malfunction", "/robot/malfunction"))
        self.topic_emergency = str(_get_param(self, "topic_emergency", "/robot/emergency"))
        self.topic_average_steering = str(_get_param(self, "topic_average_steering", "/robot/average_steering"))
        self.topic_mission_get = str(_get_param(self, "topic_mission_get", "/database/mission_get"))
        self.topic_compass = str(_get_param(self, "topic_compass", "/robot/compass"))
        self.topic_motion = str(_get_param(self, "topic_motion", "/robot/motion"))
        
        self.control_frequency = int(_get_param(self, "control_frequency", 10))
        buzzer_gpio = int(_get_param(self, "buzzer_pin", 18))
        led_pin = int(_get_param(self, "led_pin", 23))


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

        self.mission_puller = self.create_client(MissionGet, self.topic_mission_get)

        self.motion_publisher = self.create_publisher(Motion, self.topic_motion, 10)

        self.create_timer(1.0/self.control_frequency, self.update)




    def update(self) -> None:
        if self.robot_state == RobotState.MALFUNTION:
            ...
        elif self.robot_state == RobotState.EMERGENCY:
            ...
        elif self.robot_state == RobotState.MANUAL:
            self.manual_control()
        elif self.robot_state == RobotState.MISSION:
            self.autonomous_control()


    def manual_control(self):
        msg = Motion()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.arm = self.rc_ignition
        msg.throttle = self.rc_throttle
        msg.steering = self.rc_steering
        msg.reverse = self.rc_reverse
        self.motion_publisher.publish(msg)



    def autonomous_control(self):
        ...
        # görevi auto_action_node servisine gönder ve action servisini başlat
        # servis görev son noktaya ulaşınca bitecek.
        # gerektiğinde servis acilen durdurulabilecek.

    def rc_callback(self, msg: RcInput) -> None:

        if not msg.connected:
            if self.robot_state == RobotState.MANUAL or self.robot_state == RobotState.MISSION:
                self.robot_state == RobotState.IDLE
            return

        # rc açık, tüm değerleri çek
        self.rc_ignition = msg.ignition == 1
        self.rc_reverse = msg.reverse == 2

        self.fix_rc_values(msg.throttle, msg.steering)
        
        # rc arm oldu, KOŞULSUZ manual moda geç
        if self.rc_ignition:
            self.robot_state = RobotState.MANUAL
            return

        # manual moddaydın ama arm kapandı        
        if self.robot_state == RobotState.MANUAL:
            self.robot_state = RobotState.IDLE
            return

    def fix_rc_values(self, throttle:int, steering:int):
        if throttle <= self.throttle_deadband:
            self.rc_throttle = 0
        else:
            self.rc_throttle = clamp_int(throttle, 0, self.throttle_input_max)
            self.rc_throttle = map_value(self.rc_throttle, 0, self.throttle_input_max, 0, self.throttle_max)

        if steering <= self.steer_input_deadband and steering >= -self.steer_input_deadband:
            self.rc_steering = 0
        else:
            self.rc_steering = clamp_int(steering, -self.steer_input_max, self.steer_input_max)
            self.rc_steering = map_value(self.rc_steering, -self.steer_input_max, self.steer_input_max, -self.steer_max, self.steer_max)


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
                0 # response.last_waypoint, # Baştan başlasın
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
