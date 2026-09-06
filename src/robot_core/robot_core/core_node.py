#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_interfaces.action import Autonomous
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
from std_msgs.msg import Int32MultiArray, Float32, Int32
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
        self.topic_command = str(_get_param(self, "topic_command", "/user/command"))        
        self.topic_malfunction = str(_get_param(self, "topic_malfunction", "/robot/malfunction"))
        self.topic_emergency = str(_get_param(self, "topic_emergency", "/robot/emergency"))
        self.topic_mission_get = str(_get_param(self, "topic_mission_get", "/database/mission_get"))
        self.topic_motion = str(_get_param(self, "topic_motion", "/robot/motion"))
        self.topic_state = str(_get_param(self, "topic_state", "/robot/state"))
        self.action_autonomous = str(_get_param(self, "action_autonomous", "/robot/autonomous"))
        
        self.control_frequency = int(_get_param(self, "control_frequency", 10))
        buzzer_gpio = int(_get_param(self, "buzzer_pin", 18))
        led_pin = int(_get_param(self, "led_pin", 23))


        self.buzzer = Buzzer(buzzer_gpio)
        self.mission = None
        self.autonomous_task_active = False
        self.autonomous_goal_handle = None
        self.autonomous_cancel_requested = False

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

        self.rc_subs = self.create_subscription(RcInput, self.topic_rc_input, self.rc_callback, 10)
        self.command_subs = self.create_subscription(Command, self.topic_command, self.command_callback, 10)
        self.malfunction_subs = self.create_subscription(Malfunction, self.topic_malfunction, self.malfunction_callback, 10)
        self.emergency_subs = self.create_subscription(Emergency, self.topic_emergency, self.emergency_callback, 10)
        

        self.mission_puller = self.create_client(MissionGet, self.topic_mission_get)
        self.autonomous_action_client = ActionClient(
            self,
            Autonomous,
            self.action_autonomous,
        )

        self.motion_publisher = self.create_publisher(Motion, self.topic_motion, 10)
        self.state_publisher = self.create_publisher(Int32, self.topic_state, 10)


        self.create_timer(1.0/self.control_frequency, self.update)


    def update(self) -> None:
        if self.robot_state == RobotState.MALFUNTION:
            self.emergency_stop()
        elif self.robot_state == RobotState.EMERGENCY:
            self.emergency_stop()
        elif self.robot_state == RobotState.MANUAL:
            self.manual_control()
        elif self.robot_state == RobotState.MISSION:
            self.autonomous_control()

        self.publish_state()



    def publish_state(self) -> None:
        msg = Int32()
        msg.data = int(self.robot_state)
        self.state_publisher.publish(msg)


    def manual_control(self):
        msg = Motion()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.arm = self.rc_ignition
        msg.throttle = self.rc_throttle
        msg.steering = self.rc_steering
        msg.reverse = self.rc_reverse
        self.motion_publisher.publish(msg)


    def emergency_stop(self) -> None:
        msg = Motion()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.arm = False
        msg.throttle = 0
        msg.steering = 0
        msg.reverse = False
        self.motion_publisher.publish(msg)


    def rc_callback(self, msg: RcInput) -> None:
        """RC girdilerini güncelle ve robotun çalışma modunu yönet.

        ``connected`` kumanda bağlantısını, ``ignition`` ise arm durumunu
        belirtir. MISSION otonom, MANUAL kumandayla kontrol modudur.
        """
        previous_ignition = self.rc_ignition

        # Bağlantı kaybında yalnızca RC ile ilişkili çalışma modlarından çık.
        if not msg.connected:
            if self.robot_state in (RobotState.MANUAL, RobotState.MISSION):
                self.robot_state = RobotState.IDLE
            return

        # RC bağlıyken güncel kumanda değerlerini al.
        self.rc_ignition = msg.ignition == 1
        self.rc_reverse = msg.reverse == 2
        self.fix_rc_values(msg.throttle, msg.steering)

        if self.robot_state in (RobotState.IDLE, RobotState.MISSION):
            if self.rc_ignition:
                self.robot_state = RobotState.MANUAL
        elif self.robot_state == RobotState.MANUAL:
            if not self.rc_ignition:
                self.robot_state = RobotState.IDLE
        elif self.robot_state == RobotState.EMERGENCY:
            # Emergency'den çıkmak için arm kapatılıp yeniden açılmalı.
            if not previous_ignition and self.rc_ignition:
                self.robot_state = RobotState.MANUAL


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
            self.cancel_autonomous_action()
            self.get_logger().error(msg.error)


    def emergency_callback(self, msg:Emergency) -> None:
        if msg.emergency:
            self.robot_state = RobotState.EMERGENCY
            self.cancel_autonomous_action()
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
            self.robot_state = RobotState.IDLE
            self.cancel_autonomous_action()
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
            self.cancel_autonomous_action()


    def start_mission(self, mission_id: int) -> None:
        if self.robot_state != RobotState.IDLE or self.autonomous_task_active:
            self.get_logger().error("Robot beklemede değilken görev başlatılamaz!")
            return

        self.autonomous_task_active = True
        self.autonomous_cancel_requested = False

        request = MissionGet.Request()
        request.mission_id = mission_id

        future = self.mission_puller.call_async(request)
        future.add_done_callback(self._mission_received)


    def _mission_received(self, future) -> None:
        if self.robot_state != RobotState.IDLE:
            self.get_logger().warning("Robot artık beklemede değil, görev başlatılmadı")
            self._clear_autonomous_action()
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Görev alınırken servis hatası: {exc}")
            self._clear_autonomous_action()
            return

        if response is None:
            self.get_logger().error("MissionGet servisi cevap döndürmedi!")
            self._clear_autonomous_action()
            return

        if not response.raw_json:
            self.get_logger().error(f"Görev verisi boş! mission_id={response.mission_id}")
            self._clear_autonomous_action()
            return

        try:
            self.mission = Mission(
                response.raw_json,
                0.5,
                0 # response.last_waypoint, # Şimdilik hep baştan başlasın
            )
        except Exception as exc:
            self.get_logger().error(f"Mission nesnesi oluşturulamadı: {exc}")
            self._clear_autonomous_action()
            return

        if not self.mission.has_next_waypoint():
            self.get_logger().error(
                f"Görevde sonraki waypoint yok! "
                f"mission_id={response.mission_id}, "
            )
            self._clear_autonomous_action()
            return

        if not self.start_autonomous_action(response.raw_json):
            self._clear_autonomous_action()
            return

        self.robot_state = RobotState.MISSION

        self.get_logger().info(
            f"Görev başlatıldı. "
            f"mission_id={response.mission_id}, "
        )

    def start_autonomous_action(self, mission_raw: str) -> bool:
        if not self.autonomous_action_client.server_is_ready():
            self.get_logger().error("Autonomous action server hazır değil")
            return False

        goal = Autonomous.Goal()
        goal.mission_raw = mission_raw

        future = self.autonomous_action_client.send_goal_async(goal)
        future.add_done_callback(self._autonomous_goal_response)
        return True

    def _autonomous_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Autonomous action başlatılamadı: {exc}")
            self._clear_autonomous_action()
            return

        if not goal_handle.accepted:
            self.get_logger().error("Autonomous action görevi reddetti")
            self._clear_autonomous_action()
            return

        self.autonomous_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._autonomous_result_received)

        if self.autonomous_cancel_requested:
            goal_handle.cancel_goal_async()

    def _autonomous_result_received(self, future) -> None:
        cancel_requested = self.autonomous_cancel_requested

        try:
            response = future.result()
            completed = response.result.completed
        except Exception as exc:
            self.get_logger().error(f"Autonomous action sonucu alınamadı: {exc}")
            completed = False

        self._clear_autonomous_action()

        if completed:
            self.get_logger().info("Autonomous mission completed")
        elif cancel_requested:
            self.get_logger().info("Autonomous mission canceled")
        else:
            self.get_logger().warning("Autonomous mission tamamlanmadan sona erdi")

    def cancel_autonomous_action(self) -> None:
        if not self.autonomous_task_active or self.autonomous_cancel_requested:
            return

        self.autonomous_cancel_requested = True

        if self.autonomous_goal_handle is not None:
            self.autonomous_goal_handle.cancel_goal_async()

    def _clear_autonomous_action(self) -> None:
        self.autonomous_task_active = False
        self.autonomous_goal_handle = None
        self.autonomous_cancel_requested = False
        self.mission = None

        if self.robot_state == RobotState.MISSION:
            self.robot_state = RobotState.IDLE


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
