#!/usr/bin/env python3

import math
import time
from threading import Lock

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from robot_interfaces.action import Autonomous
from robot_interfaces.msg import MissionState, Gnss, Compass, Motion, Emergency, RcInput
from robot_core.Mission import Mission
from robot_core.Tools import _get_param
from robot_core.drive_log_writer import AutonomousLogger


class AutoActNode(Node):

    TO_START = "to_start"
    ON_LINE = "on_line"
    CLOSE_TO_POINT = "close_to_point"
    PIVOT_TURN = "pivot_turn"
    RECOVERY = "recovery"
    THE_END = "the_end"

    # Mesafeler metre, hizlar m/s, acilar derece. Surus ayarlari burada.
    MAX_SPEED = 1.1
    CRUISE_SPEED = 1.1
    CLOSE_SPEED = 0.6
    REACHED_DISTANCE = 0.5
    CLOSE_DISTANCE = 2.0
    LOOKAHEAD_DISTANCE = 3.0
    FRONT_ANGLE = 60.0
    MAX_STEERING = 30.0
    PIVOT_SPEED = 12.0  # Pusuladan olculen hedef donus hizi, derece/s.
    PIVOT_TOLERANCE = 5.0
    PIVOT_STOPPED_SPEED = 2.0  # Derece/s.
    PIVOT_THROTTLE_MAX = 60.0
    PIVOT_SPEED_GAIN = 1.0
    STUCK_TIME = 4.0
    RECOVERY_DISTANCE = 2.0
    GNSS_HEADING_ACCURACY = 10.0
    GNSS_ASSIST_SPEED = 0.6
    GNSS_ASSIST_STEERING = 5.0
    GNSS_ASSIST_DIFFERENCE = 10.0
    GNSS_ASSIST_WEIGHT = 0.2
    MAX_HEADING_CORRECTION = 2.0
    STOPPED_SPEED = 0.05
    BRAKE_SETTLE_TIME = 1.0
    BRAKE_RELEASE_TIME = 1.0
    START_THROTTLE = 60
    CRUISE_THROTTLE = 35
    STEERING_START_SPEED = 0.3
    CENTERED_STEERING = 2.0
    SPEED_GAIN = 4.0
    ACCELERATION_GAIN = 15.0
    THROTTLE_RISE_PER_SECOND = 8.0
    RUN_THROTTLE_MAX = 80.0
    PREDICTION_TIME = 1.0  # Logdaki gaz/fren tepkisi gecikmesini hesaba kat.
    EARTH_RADIUS = 6371000.0

    def __init__(self) -> None:
        super().__init__("auto_act_node", automatically_declare_parameters_from_overrides=True)

        self.control_frequency = float(_get_param(self, "control_frequency", 10.0))
        self.wheelbase = float(_get_param(self, "wheelbase", 1.6))
        self.throttle_deadband = int(_get_param(self, "throttle_deadband", 5))
        self.action_autonomous = str(_get_param(self, "action_autonomous", "/robot/autonomous"))
        self.topic_mission_state = str(_get_param(self, "topic_mission_state", "/robot/mission_state"))
        self.topic_gnss = str(_get_param(self, "topic_gnss", "/robot/gnss"))
        self.topic_compass = str(_get_param(self, "topic_compass", "/robot/compass"))
        self.topic_average_steering = str(_get_param(self, "topic_average_steering", "/robot/average_steering"))
        self.topic_motion = str(_get_param(self, "topic_motion", "/robot/motion"))
        self.topic_emergency = str(_get_param(self, "topic_emergency", "/robot/emergency"))
        self.drive_log_directory = str(_get_param(self, "drive_log_directory", "~/.ros/robot_drive_logs"))
        self.autonomous_logger = AutonomousLogger(self.drive_log_directory, self.get_logger())

        self.mission: Mission | None = None
        self.rc_connected = False
        self.rc_armed = False
        self.control_revoked = False
        self.cancel_requested = False
        self.goal_active = False
        self.shutdown_requested = False
        self.goal_lock = Lock()
        self.driving_mission = None

        self.average_steering = 0.0
        self.steering_angles = None
        self.last_gnss_time = self.get_clock().now()
        self.latitude = None
        self.longitude = None
        self.accuracy = None
        self.fix_ok = False
        self.speed = None
        self.speed_accuracy = None
        self.gnss_heading = None
        self.gnss_heading_accuracy = None
        self.last_compass = self.get_clock().now()
        self.compass_heading = None
        self.reset_drive()

        self.rc_subs = self.create_subscription(RcInput, str(_get_param(self, "topic_rc_input", "/rc/input")), self.rc_callback, 10)
        self.gnss_subs = self.create_subscription(Gnss, self.topic_gnss, self.gnss_callback, 10)
        self.compass_subs = self.create_subscription(Compass, self.topic_compass, self.compass_callback, 10)
        self.steering_angles_subs = self.create_subscription(Float32MultiArray, str(_get_param(self, "topic_steering_angles", "/robot/steering_angles")), self.steering_angles_callback, 10)
        self.av_steer_subs = self.create_subscription(Float32, self.topic_average_steering, self.av_steer_callback, 10)
        self.mission_state_publisher = self.create_publisher(MissionState, self.topic_mission_state, 10)
        self.motion_publisher = self.create_publisher(Motion, self.topic_motion, 10)
        self.emergency_publisher = self.create_publisher(Emergency, self.topic_emergency, 10)

        self.action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, Autonomous, self.action_autonomous, execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=self.action_callback_group)
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.update)

    def reset_drive(self):
        """Yeni gorevi onceki gorevin hiz ve manevra bilgilerinden ayirir."""
        self.phase = self.TO_START
        self.arm = False
        self.stop_steering = True
        self.coast = False
        self.brake_since = None
        self.throttle = 0
        self.steering = 0
        self.reverse = False
        self.target_speed = 0.0
        self.current_target = None
        self.previous_waypoint = None
        self.next_wp_distance = 0.0
        self.last_speed = None
        self.accelerations = []
        self.acceleration = 0.0
        self.driving = False
        self.launching = False
        self.launch_since = None
        self.cruise_throttle = float(self.CRUISE_THROTTLE)
        self.pivot = 0
        self.pivot_stage = ""
        self.pivot_direction = 0
        self.pivot_heading = None
        self.pivot_started = None
        self.pivot_throttle = 0.0
        self.last_turn_heading = None
        self.turn_rates = []
        self.turn_rate = 0.0
        self.stuck_since = None
        self.mission_has_moved = False
        self.recovery_phase = None
        self.recovery_reverse = False
        self.recovery_steering = 0.0
        self.recovery_start = None
        self.recovery_stopping = False
        self.finish_requested = False
        self.mission_error = ""
        self.aim_point = None
        self.aim_heading_error = None
        self.sensor_heading_difference = None
        self.gnss_body_heading = None
        self.heading_correction = 0.0
        self.navigation_heading = None

    def update(self) -> None:
        """10 Hz: verileri oku, fazi ilerlet, Motion ve gorev durumunu yayinla."""
        # Action kapanirken bir kontrol frame'i yeniden hareket baslatamasin.
        with self.goal_lock:
            try:
                self.update_drive()
            except Exception as exc:
                self.fail_mission(f"Otonom surus hatasi: {exc}")
                self.log_drive("error")
                self.autonomous_logger.close()

    def update_drive(self):
        """Aktif gorevin bir surus frame'ini isler."""
        mission = self.mission
        if mission is None:
            if self.driving_mission is not None:
                self.stop()
                self.publish_motion()
                self.log_drive("mission_ended")
                self.autonomous_logger.close()
                self.reset_drive()
                self.driving_mission = None
            return

        if mission is not self.driving_mission:
            self.reset_drive()
            self.driving_mission = mission
            self.arm = True
            self.autonomous_logger.start(mission.cloud_id)

        if self.mission_error:
            return
        if self.control_revoked or self.cancel_requested:
            self.stop()
            self.publish_motion()
            return

        self.aim_point = None
        self.aim_heading_error = None

        self.current_target = mission.get_current_target()
        self.previous_waypoint = mission.get_previous_waypoint()
        self.next_wp_distance = None
        if self.latitude is None or self.longitude is None or self.compass_heading is None or self.speed is None:
            self.stop()
            self.publish_motion()
            self.log_drive("waiting_for_data")
            return

        self.update_acceleration()
        self.update_turn_rate()
        self.next_wp_distance = mission.distance_to_target(self.latitude, self.longitude)
        self.update_heading()

        if mission.is_completed:
            self.phase = self.THE_END

        if self.phase != self.PIVOT_TURN and self.phase != self.RECOVERY and self.phase != self.THE_END:
            self.advance_waypoint()
            if self.phase != self.THE_END:
                self.decide_phase()

        if self.phase == self.TO_START:
            self.go_to_point(self.point_vector(self.current_target), self.CRUISE_SPEED)
        elif self.phase == self.ON_LINE:
            self.follow_line()
        elif self.phase == self.CLOSE_TO_POINT:
            self.approach_point()
        elif self.phase == self.PIVOT_TURN:
            self.turn_in_place()
        elif self.phase == self.RECOVERY:
            self.recover()
        elif self.phase == self.THE_END:
            self.stop()

        self.check_stuck()
        mission_complete = self.finish_requested and self.stop_complete() and mission is self.mission
        if mission_complete:
            self.arm = False
            self.reverse = False
        self.publish_motion()
        # Action dongusu tamamlanmayi gormeden once sifir Motion yayinlanmis olur.
        if mission_complete:
            mission.mark_completed()
        self.log_drive()
        if mission.is_completed or self.mission_error:
            self.autonomous_logger.close()
        self.publish_mission_state()

    def advance_waypoint(self):
        """Yalnizca 0.5 m kabul dairesine girilen waypoint'leri ilerletir."""
        mission = self.driving_mission
        while self.next_wp_distance <= self.REACHED_DISTANCE:
            if mission.index == mission.waypoint_count - 1:
                self.finish_requested = True
                self.phase = self.THE_END
                return
            mission.set_index(mission.index + 1)
            self.current_target = mission.get_current_target()
            self.previous_waypoint = mission.get_previous_waypoint()
            self.next_wp_distance = mission.distance_to_target(self.latitude, self.longitude)

    def decide_phase(self):
        """Ilk noktaya direkt gidilir; digerlerinde cizgi izlenir."""
        if self.next_wp_distance <= self.CLOSE_DISTANCE:
            self.phase = self.CLOSE_TO_POINT
        elif self.previous_waypoint is None:
            self.phase = self.TO_START
        else:
            self.phase = self.ON_LINE

    def go_to_point(self, point, speed):
        """Verilen dogu/kuzey vektorune surer; hedef arkadaysa manevraya girer."""
        self.aim_point = point
        error = self.heading_error(point)
        if self.current_target is not None:
            target_error = self.heading_error(self.point_vector(self.current_target))
            if abs(target_error) > self.FRONT_ANGLE:
                error = target_error
                self.aim_point = self.point_vector(self.current_target)
        self.aim_heading_error = error
        if abs(error) > self.FRONT_ANGLE:
            self.phase = self.PIVOT_TURN
            self.pivot_stage = "stopping"
            # Anten donuste merkez etrafinda yer degistirir; hedef yonu sabit tut.
            self.pivot_heading = math.degrees(math.atan2(self.aim_point[0], self.aim_point[1])) % 360.0
            self.pivot_direction = 1 if error > 0.0 else -1
            self.stop()
            return

        # Mevcut teker acisiyla bir frame'de donulecek aciyi hesaba kat.
        wheel_angle = math.radians(max(-self.MAX_STEERING, min(self.MAX_STEERING, self.average_steering)))
        heading_change = math.degrees(2.0 * self.speed * math.tan(wheel_angle) / self.wheelbase / self.control_frequency)
        steering = (error - heading_change) / 2.0  # 4WS: 20 derece icin steering 10.
        self.drive(speed, steering, False)

    def follow_line(self):
        """Onceki waypoint ile hedef arasindaki cizgide 3 m hayali noktayi izler."""
        self.go_to_point(self.line_point(), self.CRUISE_SPEED)

    def approach_point(self):
        """Son 2 m'yi 0.6 m/s ile gider; uygun sonraki noktaya donusu baslatir."""
        self.go_to_point(self.close_point(), self.CLOSE_SPEED)

    def line_point(self):
        """Hipotenus 3 m: ilerleme = sqrt(3^2 - cizgiye_uzaklik^2)."""
        end_x, end_y = self.point_vector(self.current_target)
        if self.previous_waypoint is None:
            return end_x, end_y
        start_x, start_y = self.point_vector(self.previous_waypoint)
        dx = end_x - start_x
        dy = end_y - start_y
        length = math.hypot(dx, dy)
        if length < 0.001:
            return end_x, end_y

        ux = dx / length
        uy = dy / length
        projection = -start_x * ux - start_y * uy
        foot_x = start_x + projection * ux
        foot_y = start_y + projection * uy
        cross_distance = math.hypot(foot_x, foot_y)
        # 3 m'den uzakta cizgi ile 3 m yaricapli cember kesismez: izdusume git.
        ahead = math.sqrt(max(0.0, self.LOOKAHEAD_DISTANCE ** 2 - cross_distance ** 2))
        along = max(0.0, min(length, projection + ahead))
        # Segment uclarinda hayali nokta waypoint'e kisaltilir; hedef atlanmaz.
        return start_x + along * ux, start_y + along * uy

    def close_point(self):
        """Donus hedefini kabul dairesi icinde tutarak waypoint'in atlanmasini onler."""
        point = self.point_vector(self.current_target)
        mission = self.driving_mission
        if mission.index + 1 >= mission.waypoint_count:
            return point
        next_point = self.point_vector(mission.waypoints[mission.index + 1])
        if abs(self.heading_error(next_point)) > self.FRONT_ANGLE:
            return point
        dx = next_point[0] - point[0]
        dy = next_point[1] - point[1]
        length = math.hypot(dx, dy)
        if length < 0.001:
            return point
        turn_distance = min(length, self.REACHED_DISTANCE / 2.0) * max(0.0, 1.0 - self.next_wp_distance / self.CLOSE_DISTANCE)
        turn_point = (point[0] + dx / length * turn_distance, point[1] + dy / length * turn_distance)
        if abs(self.heading_error(turn_point)) > self.FRONT_ANGLE:
            return point
        return turn_point

    def update_turn_rate(self):
        """GNSS yerine pusulanin son bes frame'deki donus hizini kullanir."""
        if self.last_turn_heading is not None:
            change = (self.compass_heading - self.last_turn_heading + 180.0) % 360.0 - 180.0
            self.turn_rates.append(change * self.control_frequency)
            self.turn_rates = self.turn_rates[-5:]
            self.turn_rate = sum(self.turn_rates) / len(self.turn_rates)
        self.last_turn_heading = self.compass_heading

    def wheels_ready(self, pivot=False):
        """X duzeninde ortalama aci sifirdir; dort teker ayri kontrol edilir."""
        if self.steering_angles is None:
            return False
        targets = [45.0, -45.0, -45.0, 45.0] if pivot else [0.0, 0.0, 0.0, 0.0]
        return all(abs(angle - target) <= self.CENTERED_STEERING for angle, target in zip(self.steering_angles, targets))

    def turn_in_place(self):
        """Dur, tekerleri X yap, pusulayla don, frenle ve tekerleri merkezle."""
        error = (self.pivot_heading - self.compass_heading + 180.0) % 360.0 - 180.0
        self.aim_heading_error = error
        self.arm = True
        now = time.monotonic()

        if self.pivot_stage == "stopping":
            self.stop()
            if self.stop_complete() and self.wheels_ready():
                self.reverse = False
                self.pivot_stage = "aligning"
            return

        if self.pivot_stage == "aligning":
            self.stop()
            self.pivot = self.pivot_direction
            if self.wheels_ready(True):
                self.pivot_stage = "turning"
                self.pivot_started = now
                self.pivot_throttle = float(self.CRUISE_THROTTLE)
                self.coast = True
            return

        if self.pivot_stage == "turning":
            self.pivot = self.pivot_direction
            self.coast = True
            rotation_speed = self.turn_rate * self.pivot_direction
            stopping_angle = self.PIVOT_TOLERANCE + max(0.0, rotation_speed) * self.PREDICTION_TIME
            if abs(error) <= stopping_angle or error * self.pivot_direction <= 0.0:
                self.pivot_stage = "braking"
                self.stop()
                self.pivot = self.pivot_direction
                return
            if now - self.pivot_started < self.BRAKE_RELEASE_TIME:
                self.throttle = 0
                return
            target_rate = min(self.PIVOT_SPEED, max(self.PIVOT_STOPPED_SPEED * 2.0, abs(error) / 2.0))
            adjustment = self.PIVOT_SPEED_GAIN * (target_rate - rotation_speed) / self.control_frequency
            adjustment = max(-2.0, min(self.THROTTLE_RISE_PER_SECOND / self.control_frequency, adjustment))
            self.pivot_throttle = max(0.0, min(self.PIVOT_THROTTLE_MAX, self.pivot_throttle + adjustment))
            self.throttle = int(round(self.pivot_throttle)) if rotation_speed < self.PIVOT_SPEED else 0
            return

        if self.pivot_stage == "braking":
            self.stop()
            self.pivot = self.pivot_direction  # Fren kapanirken tekerleri ve yon rolelerini tut.
            if not self.stop_complete(False) or abs(self.turn_rate) > self.PIVOT_STOPPED_SPEED:
                return
            if abs(error) > self.PIVOT_TOLERANCE * 2.0:
                self.pivot_direction = 1 if error > 0.0 else -1
                self.pivot_stage = "aligning"
            else:
                self.pivot_stage = "centering"
            return

        if self.pivot_stage == "centering":
            self.stop()
            if self.stop_complete() and self.wheels_ready():
                self.pivot_stage = ""
                self.decide_phase()

    def check_stuck(self):
        """Ilk hareketten sonra, gazla dort saniye kesintisiz durulursa kurtarir."""
        if self.phase == self.PIVOT_TURN:
            if self.pivot_stage != "turning" or self.throttle < self.throttle_deadband or not self.arm or abs(self.turn_rate) > self.PIVOT_STOPPED_SPEED:
                self.stuck_since = None
                return
            if self.stuck_since is None:
                self.stuck_since = time.monotonic()
            if time.monotonic() - self.stuck_since >= self.STUCK_TIME:
                self.fail_mission("Eksenel donus basarisiz: gaz verilmesine ragmen 4 saniye donus algilanamadi")
            return
        if self.speed > self.STOPPED_SPEED:
            self.mission_has_moved = True
        if not self.mission_has_moved or self.throttle < self.throttle_deadband or not self.arm or self.speed > self.STOPPED_SPEED:
            self.stuck_since = None
            return
        now = time.monotonic()
        if self.stuck_since is None:
            self.stuck_since = now
        if now - self.stuck_since < self.STUCK_TIME:
            return
        if self.phase == self.RECOVERY:
            self.fail_mission("Kurtarma basarisiz: aksi yone gaz verilmesine ragmen 4 saniye hareket edilemedi")
            return
        self.recovery_phase = self.phase
        self.recovery_reverse = not self.reverse
        self.recovery_steering = self.average_steering
        self.recovery_start = None
        self.recovery_stopping = False
        self.phase = self.RECOVERY
        self.stop()
        self.get_logger().info(f"Robot 4 saniye hareket edemedi; 2 m aksi yone kurtarma basliyor. donus_fazi={self.recovery_phase}, reverse={self.recovery_reverse}")

    def recover(self):
        """Dur, mevcut teker acisiyla 2 m aksi yone git, dur ve onceki faza don."""
        if self.recovery_start is None:
            self.stop()
            if not self.stop_complete():
                return
            self.recovery_start = (self.latitude, self.longitude)

        if self.recovery_stopping:
            self.stop()
            if self.stop_complete():
                self.phase = self.recovery_phase
                self.recovery_phase = None
                self.recovery_start = None
                self.get_logger().info(f"Kurtarma tamamlandi; suruse devam ediliyor. faz={self.phase}")
            return

        x, y = self.position_vector(self.recovery_start[0], self.recovery_start[1])
        if math.hypot(x, y) + self.speed * self.PREDICTION_TIME >= self.RECOVERY_DISTANCE:
            self.recovery_stopping = True
            self.stop()
            return
        self.drive(self.CLOSE_SPEED, self.recovery_steering, self.recovery_reverse)

    def update_acceleration(self):
        """Son bes kontrol frame'indeki hiz degisimlerinin ortalamasini alir."""
        if self.last_speed is not None:
            self.accelerations.append((self.speed - self.last_speed) * self.control_frequency)
            self.accelerations = self.accelerations[-5:]
        self.last_speed = self.speed
        if self.accelerations:
            self.acceleration = sum(self.accelerations) / len(self.accelerations)

    def update_heading(self):
        """Pusula ana kaynaktir; duz ve duzenli suruste GNSS kucuk bir duzeltme yapar."""
        self.sensor_heading_difference = None
        self.gnss_body_heading = None
        self.navigation_heading = self.compass_heading
        if self.gnss_heading is None or self.gnss_heading_accuracy is None or self.compass_heading is None:
            self.heading_correction = 0.0
            return
        # headMot geri giderken arka tarafi gosterir; govde yonune cevir.
        self.gnss_body_heading = (self.gnss_heading + (180.0 if self.reverse else 0.0)) % 360.0
        difference = (self.gnss_body_heading - self.compass_heading + 180.0) % 360.0 - 180.0
        self.sensor_heading_difference = abs(difference)
        steady = self.speed is not None and self.speed >= self.GNSS_ASSIST_SPEED and abs(self.acceleration) <= 0.2
        straight = abs(self.average_steering) <= self.GNSS_ASSIST_STEERING and abs(self.steering) <= self.GNSS_ASSIST_STEERING
        reliable = self.fix_ok and 0.0 <= self.gnss_heading_accuracy < self.GNSS_HEADING_ACCURACY and abs(difference) <= self.GNSS_ASSIST_DIFFERENCE
        if not steady or not straight or not reliable or self.reverse or self.phase in (self.PIVOT_TURN, self.RECOVERY, self.THE_END):
            self.heading_correction = 0.0
            return
        correction = max(-self.MAX_HEADING_CORRECTION, min(self.MAX_HEADING_CORRECTION, difference * self.GNSS_ASSIST_WEIGHT))
        self.heading_correction += 0.1 * (correction - self.heading_correction)
        self.navigation_heading = (self.compass_heading + self.heading_correction) % 360.0

    def drive(self, speed, steering, reverse):
        """Hiz kontrolunu ve hareketle birlikte baslayan dumenlemeyi uygular."""
        self.target_speed = max(0.0, min(self.MAX_SPEED, speed))
        self.reverse = reverse
        self.arm = True
        centered = abs(self.average_steering) <= self.CENTERED_STEERING
        if self.steering_angles is not None:
            centered = centered and self.wheels_ready()
        if not self.driving and self.speed <= self.STEERING_START_SPEED and not centered:
            self.stop()
            return
        self.throttle = self.speed_throttle()
        self.steering = int(round(max(-self.MAX_STEERING, min(self.MAX_STEERING, steering))))
        if self.launching or self.speed <= self.STEERING_START_SPEED:
            self.steering = 0
        if self.throttle >= self.throttle_deadband:
            self.brake_since = None
        elif not self.coast:
            self.steering = 0

    def speed_throttle(self):
        """Freni birak, duz tekerle yumusak kalk; seyir hizini yalniz gazla ayarla."""
        self.coast = True
        now = time.monotonic()
        if not self.driving:
            self.driving = True
            self.launching = self.speed <= self.STEERING_START_SPEED
            self.launch_since = now if self.launching else None
            self.cruise_throttle = float(self.CRUISE_THROTTLE)

        if self.launching:
            if now - self.launch_since < self.BRAKE_RELEASE_TIME:
                return 0
            if self.speed <= self.STEERING_START_SPEED:
                # Darbe veya 60 -> 80 sicramasi yok; hareket gorulene kadar gaz yavas artar.
                self.cruise_throttle = min(self.START_THROTTLE, self.cruise_throttle + self.THROTTLE_RISE_PER_SECOND / self.control_frequency)
                return int(round(self.cruise_throttle))
            self.launching = False

        # Ivme hizdaki degisimi sonumler. Her duzeltme saniye bazlidir; frame basina yigilmaz.
        adjustment = (self.SPEED_GAIN * (self.target_speed - self.speed) - self.ACCELERATION_GAIN * self.acceleration) / self.control_frequency
        adjustment = max(-2.0, min(self.THROTTLE_RISE_PER_SECOND / self.control_frequency, adjustment))
        self.cruise_throttle = max(0.0, min(self.RUN_THROTTLE_MAX, self.cruise_throttle + adjustment))
        if self.speed >= self.MAX_SPEED:
            return 0  # Hiz duzenlemesi mekanik freni calistirmaz; durus stop() ile yapilir.
        return int(round(self.cruise_throttle))

    def stop(self):
        """Roleleri acik ve mevcut yonde tutar; throttle sifir komutu freni sikar."""
        self.pivot = 0
        self.target_speed = 0.0
        self.throttle = 0
        self.coast = False
        self.steering = 0
        self.driving = False
        self.launching = False
        self.launch_since = None
        self.stuck_since = None
        if self.brake_since is None:
            self.brake_since = time.monotonic()

    def stop_complete(self, check_steering=True):
        """Frenin kapanma suresi dolmadan veya robot durmadan yon degistirilmez."""
        centered = abs(self.average_steering) <= self.CENTERED_STEERING
        if self.steering_angles is not None:
            centered = self.wheels_ready()
        return self.speed is not None and self.speed <= self.STOPPED_SPEED and (centered or not check_steering) and self.brake_since is not None and time.monotonic() - self.brake_since >= self.BRAKE_SETTLE_TIME

    def position_vector(self, latitude, longitude):
        """GNSS konumunu robota gore yerel metreye cevirir: x dogu, y kuzey."""
        longitude_difference = (longitude - self.longitude + 180.0) % 360.0 - 180.0
        x = self.EARTH_RADIUS * math.radians(longitude_difference) * math.cos(math.radians(self.latitude))
        y = self.EARTH_RADIUS * math.radians(latitude - self.latitude)
        return x, y

    def point_vector(self, waypoint):
        return self.position_vector(waypoint.latitude, waypoint.longitude)

    def heading_error(self, point):
        """Pusula gibi kuzey 0, dogu 90; pozitif hata saga donustur."""
        heading = math.degrees(math.atan2(point[0], point[1]))
        current_heading = self.compass_heading if self.navigation_heading is None else self.navigation_heading
        return (heading - current_heading + 180.0) % 360.0 - 180.0

    def publish_motion(self):
        """Hesaplanan komutu yayinlar; iptal edilmis gorev hareket uretemez."""
        if self.mission is None or self.mission is not self.driving_mission or self.mission.is_completed or self.shutdown_requested or self.mission_error or self.control_revoked or self.cancel_requested:
            self.arm = False
            self.throttle = 0
            self.steering = 0
            self.reverse = False
            self.coast = False
            self.pivot = 0
        if self.throttle < self.throttle_deadband and not self.coast:
            if self.brake_since is None:
                self.brake_since = time.monotonic()
        else:
            self.brake_since = None
        # Dururken dumen motorlarini kapatmak icin hubmotor rolesi kullanilmaz.
        if self.target_speed == 0.0 or self.launching or self.speed is None or self.speed <= self.STEERING_START_SPEED:
            self.steering = 0
        # Sifir aci komutunu gerceklestirmek icin dumen motorlari dururken de etkin kalir.
        self.stop_steering = not self.arm
        msg = Motion()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.arm = self.arm
        msg.throttle = self.throttle
        msg.steering = self.steering
        msg.reverse = self.reverse
        msg.stop_steering = self.stop_steering
        msg.coast = self.coast
        msg.autonomous = True
        msg.pivot = self.pivot
        self.motion_publisher.publish(msg)

    def fail_mission(self, reason):
        """Robotu durdurur, nedeni yazdirir ve core_node'a Emergency bildirir."""
        if self.mission_error:
            return
        self.mission_error = reason
        self.finish_requested = False
        self.phase = self.THE_END
        self.stop()
        self.publish_motion()
        msg = Emergency()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.emergency = True
        msg.error = reason
        self.get_logger().error(reason)
        self.emergency_publisher.publish(msg)

    def end_drive(self, event):
        """Action sonucu donmeden once son sifir komutunu yazar ve logu kapatir."""
        self.mission = None
        self.stop()
        self.publish_motion()
        self.log_drive(event)
        self.autonomous_logger.close()
        self.driving_mission = None

    def log_drive(self, event="update"):
        """Olcumleri, karar verilen hedefi ve yayinlanan komutu ayni satira yazar."""
        mission = self.driving_mission
        if mission is None:
            return
        if event == "update":
            if self.mission_error:
                event = "error"
            elif mission.is_completed:
                event = "completed"
            elif self.mission is not mission:
                event = "mission_ended"

        previous = self.previous_waypoint
        target = self.current_target
        next_point = None
        if mission.index + 1 < mission.waypoint_count:
            next_point = mission.waypoints[mission.index + 1]
        cross_track_error = None
        if self.latitude is not None and self.longitude is not None:
            if previous is not None and target is not None:
                start_x, start_y = self.point_vector(previous)
                end_x, end_y = self.point_vector(target)
                dx = end_x - start_x
                dy = end_y - start_y
                length = math.hypot(dx, dy)
                if length > 0.001:
                    cross_track_error = (dy * start_x - dx * start_y) / length  # Pozitif: cizginin solu.

        recovery_distance = None
        if self.phase == self.RECOVERY and self.recovery_start is not None and self.latitude is not None and self.longitude is not None:
            x, y = self.position_vector(self.recovery_start[0], self.recovery_start[1])
            recovery_distance = math.hypot(x, y)

        values = {
            "event": event,
            "error": self.mission_error,
            "mission_id": mission.cloud_id,
            "phase": self.phase,
            "waypoint_index": mission.index,
            "waypoint_count": mission.waypoint_count,
            "completion_rate": mission.completion_rate,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "gnss_accuracy_m": self.accuracy,
            "gnss_fix_ok": self.fix_ok,
            "gnss_speed_accuracy_m_s": self.speed_accuracy,
            "compass_heading_deg": self.compass_heading,
            "gnss_heading_deg": self.gnss_heading,
            "gnss_body_heading_deg": self.gnss_body_heading,
            "navigation_heading_deg": self.navigation_heading,
            "heading_correction_deg": self.heading_correction,
            "gnss_heading_accuracy_deg": self.gnss_heading_accuracy,
            "sensor_heading_difference_deg": self.sensor_heading_difference,
            "previous_latitude": previous.latitude if previous is not None else None,
            "previous_longitude": previous.longitude if previous is not None else None,
            "target_latitude": target.latitude if target is not None else None,
            "target_longitude": target.longitude if target is not None else None,
            "next_latitude": next_point.latitude if next_point is not None else None,
            "next_longitude": next_point.longitude if next_point is not None else None,
            "target_distance_m": self.next_wp_distance if target is not None else None,
            "aim_east_m": self.aim_point[0] if self.aim_point is not None else None,
            "aim_north_m": self.aim_point[1] if self.aim_point is not None else None,
            "heading_error_deg": self.aim_heading_error,
            "cross_track_error_m": cross_track_error,
            "speed_m_s": self.speed,
            "target_speed_m_s": self.target_speed,
            "acceleration_m_s2": self.acceleration,
            "predicted_speed_m_s": self.speed + max(0.0, self.acceleration) * self.PREDICTION_TIME if self.speed is not None else None,
            "average_steering_deg": self.average_steering,
            "rc_connected": self.rc_connected,
            "rc_armed": self.rc_armed,
            "control_revoked": self.control_revoked,
            "arm": self.arm,
            "coast": self.coast,
            "stop_steering": self.stop_steering,
            "brake_elapsed_s": time.monotonic() - self.brake_since if self.brake_since is not None else 0.0,
            "throttle": self.throttle,
            "steering_deg": self.steering,
            "reverse": self.reverse,
            "pivot": self.pivot,
            "pivot_stage": self.pivot_stage,
            "pivot_heading_deg": self.pivot_heading if self.phase == self.PIVOT_TURN else None,
            "turn_rate_deg_s": self.turn_rate,
            "pivot_throttle": self.pivot_throttle if self.phase == self.PIVOT_TURN else None,
            "steering_front_left_deg": self.steering_angles[0] if self.steering_angles is not None else None,
            "steering_front_right_deg": self.steering_angles[1] if self.steering_angles is not None else None,
            "steering_rear_left_deg": self.steering_angles[2] if self.steering_angles is not None else None,
            "steering_rear_right_deg": self.steering_angles[3] if self.steering_angles is not None else None,
            "launching": self.launching,
            "launch_elapsed_s": time.monotonic() - self.launch_since if self.launch_since is not None else None,
            "cruise_throttle": self.cruise_throttle,
            "stuck_duration_s": time.monotonic() - self.stuck_since if self.stuck_since is not None else 0.0,
            "mission_has_moved": self.mission_has_moved,
            "recovery_phase": self.recovery_phase,
            "recovery_reverse": self.recovery_reverse if self.recovery_phase is not None else None,
            "recovery_distance_m": recovery_distance,
        }
        self.autonomous_logger.log(values)


# region Subscriptions

    def rc_callback(self, msg):
        with self.goal_lock:
            self.rc_connected = bool(msg.connected)
            self.rc_armed = msg.connected and msg.ignition == 1
            if not self.rc_connected or self.rc_armed:
                self.control_revoked = True
                if self.mission is not None:
                    self.stop()
                    self.publish_motion()

    def gnss_callback(self, msg: Gnss) -> None:
        self.last_gnss_time = msg.header.stamp
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.accuracy = msg.accuracy_m
        self.fix_ok = msg.fix_ok
        self.speed = msg.ground_speed_m_s
        self.speed_accuracy = msg.speed_accuracy_m_s
        self.gnss_heading = msg.heading_deg
        self.gnss_heading_accuracy = msg.heading_accuracy_deg


    def compass_callback(self, msg: Compass) -> None:
        self.last_compass = msg.header.stamp
        self.compass_heading = msg.heading_deg


    def steering_angles_callback(self, msg):
        if len(msg.data) == 4:
            self.steering_angles = list(msg.data)

    def av_steer_callback(self, msg: Float32) -> None:
        self.average_steering = float(msg.data)
        

# endregion

# region ACTION STUFF

    def goal_callback(self, goal_request: Autonomous.Goal) -> GoalResponse:
        if not goal_request.mission_raw.strip():
            self.get_logger().warning("Empty autonomous mission rejected")
            return GoalResponse.REJECT

        with self.goal_lock:
            if not self.rc_connected or self.rc_armed:
                self.get_logger().warning("RC bagli ve arm kapali olmadan gorev kabul edilmez")
                return GoalResponse.REJECT
            if self.goal_active:
                self.get_logger().warning(
                    "An autonomous mission is already active"
                )
                return GoalResponse.REJECT

            self.control_revoked = False
            self.cancel_requested = False
            self.goal_active = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Autonomous mission cancel requested")
        with self.goal_lock:
            self.cancel_requested = True
            self.stop()
            self.publish_motion()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> Autonomous.Result:
        result = Autonomous.Result()
        result.completed = False

        try:
            with self.goal_lock:
                self.mission_error = ""
            mission = Mission(goal_handle.request.mission_raw)
            with self.goal_lock:
                self.mission = mission

            self.get_logger().info(f"Autonomous mission started. mission_id={mission.cloud_id}")
            wait_period = 1.0 / self.control_frequency

            while rclpy.ok() and not self.shutdown_requested and not mission.is_completed and not self.mission_error:
                if goal_handle.is_cancel_requested:
                    break
                time.sleep(wait_period)

            with self.goal_lock:
                # Bitis aninda gelen iptal de basari olarak bildirilmemeli.
                if goal_handle.is_cancel_requested:
                    self.end_drive("error" if self.mission_error else "canceled")
                    goal_handle.canceled()
                    self.get_logger().info("Autonomous mission canceled")
                elif not rclpy.ok() or self.shutdown_requested or self.mission_error:
                    self.end_drive("error" if self.mission_error else "shutdown")
                    goal_handle.abort()
                else:
                    self.publish_mission_state()
                    self.end_drive("completed")
                    goal_handle.succeed()
                    result.completed = True
                    self.get_logger().info("Autonomous mission completed")
            return result

        except Exception as exc:
            with self.goal_lock:
                self.fail_mission(f"Autonomous mission error: {exc}")
                self.end_drive("error")
                if goal_handle.is_active:
                    goal_handle.abort()
            return result

        finally:
            with self.goal_lock:
                self.mission = None
                self.goal_active = False

# endregion

    def publish_mission_state(self) -> None:
        mission = self.mission
        if mission is None:
            return

        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = mission.cloud_id
        msg.completion_rate = mission.completion_rate
        self.mission_state_publisher.publish(msg)

    def destroy_node(self) -> bool:
        self.shutdown_requested = True
        self.publish_motion()
        self.log_drive("shutdown")
        self.autonomous_logger.close()
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = AutoActNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
