#!/usr/bin/env python3

import time
from threading import Lock

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float32, Int32
from robot_interfaces.action import Autonomous
from robot_interfaces.msg import MissionState, Gnss, Compass, Motion
from robot_core.Mission import Mission
from robot_core.Tools import _get_param, RobotState


class AutoActNode(Node):

    TO_START = "to_start"
    TO_LINE = "to_line"
    LINE_FOLLOW = "line_follow"
    CLOSE_TO_WP = "close_to_wp"
    MANEUVER = "maneuver"
    TO_END = "to_end"




    def __init__(self) -> None:
        super().__init__(
            "auto_act_node",
            automatically_declare_parameters_from_overrides=True,
        )

        self.control_frequency = float(_get_param(self, "control_frequency", 10.0))
        self.action_autonomous = str(_get_param(self, "action_autonomous", "/robot/autonomous"))
        self.topic_mission_state = str(_get_param(self, "topic_mission_state", "/robot/mission_state"))
        self.topic_gnss = str(_get_param(self, "topic_gnss", "/robot/gnss"))
        self.topic_compass = str(_get_param(self, "topic_compass", "/robot/compass"))
        self.topic_average_steering = str(_get_param(self, "topic_average_steering", "/robot/average_steering",))
        

        self.mission: Mission | None = None
        self.goal_active = False
        self.shutdown_requested = False
        self.goal_lock = Lock()

        self.average_steering = 0.0
        self.sufficient_distance: float = 1.0
        self.next_wp_distance: float = 999.0

        self.last_gnss_time = self.get_clock().now()
        self.latitude: float | None = None
        self.longitude: float | None = None
        self.accuracy: float | None = None
        self.fix_ok: bool = False
        self.speed: float | None = None
        self.speed_accuracy: float | None = None
        self.gnss_heading : float | None = None
        self.gnss_heading_accuracy : float | None = None
        self.last_compass = self.get_clock().now()
        self.compass_heading: float | None = None


        self.gnss_subs = self.create_subscription(Gnss, self.topic_gnss, self.gnss_callback, 10)
        self.compass_subs = self.create_subscription(Compass, self.topic_compass, self.compass_callback, 10)
        self.av_steer_subs = self.create_subscription(Float32, self.topic_average_steering, self.av_steer_callback, 10)
        self.mission_state_publisher = self.create_publisher(MissionState, self.topic_mission_state, 10)
        self.motion_publisher = self.create_publisher(Motion, self.topic_motion, 10)

        self.action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            Autonomous,
            self.action_autonomous,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group,
        )

        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.update,)

        self.phase = self.TO_START



    def update(self) -> None:
        if self.mission is None:
            return

        # eldeki veri: mission, gnss, compass, av_steering, state
        # üretilecek veri motion
        # fazlar: 
        #   TO_START => self.mission.get_previous_waypoint() == None ; görev yeni başladı. move_to_point(mission)
        self.sufficient_distance: float = 1.0
        self.current_target = self.mission.get_current_target()
        self.previous_waypoint = self.mission.get_previous_waypoint()
        self.current_line = self.create_line(self.previous_waypoint, self.current_target)

        self.next_wp_distance = self.mission.distance_to_target(self.latitude, self.longitude, self.mission.index)

        self.decide_phase()

        if self.phase == self.TO_START:
            self.move_to_point()

        self.publish_motion() # TODO doldur
        self.publish_mission_state()

    def decide_phase(self):
        
        if self.phase == self.TO_START:
            if self.mission.advance_if_target_reached(self.latitude, self.longitude):
                self.phase = self.LINE_FOLLOW
            

        if self.phase == self.LINE_FOLLOW:
            if self.far_from_line():
                self.phase == self.TO_LINE

            if self.mission.advance_if_target_reached(self.latitude, self.longitude):
                self.phase == self.LINE_FOLLOW

        



    



# region Subscriptions

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


    def av_steer_callback(self, msg: Float32) -> None:
        self.average_steering = float(msg.data)
        

# endregion

# region ACTION STUFF

    def goal_callback(self, goal_request: Autonomous.Goal) -> GoalResponse:
        if not goal_request.mission_raw.strip():
            self.get_logger().warning("Empty autonomous mission rejected")
            return GoalResponse.REJECT

        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warning(
                    "An autonomous mission is already active"
                )
                return GoalResponse.REJECT

            self.goal_active = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Autonomous mission cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> Autonomous.Result:
        result = Autonomous.Result()
        result.completed = False

        try:
            try:
                self.mission = Mission(goal_handle.request.mission_raw)
            except Exception as exc:
                self.get_logger().error(f"Invalid autonomous mission: {exc}")
                goal_handle.abort()
                return result

            self.get_logger().info(
                "Autonomous mission started. "
                f"mission_id={self.mission.cloud_id}"
            )

            wait_period = 1.0 / self.control_frequency

            while (
                rclpy.ok()
                and not self.shutdown_requested
                and not self.mission.is_completed
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Autonomous mission canceled")
                    return result

                time.sleep(wait_period)

            if not rclpy.ok() or self.shutdown_requested:
                goal_handle.abort()
                return result

            self.publish_mission_state()
            goal_handle.succeed()
            result.completed = True
            self.get_logger().info("Autonomous mission completed")
            return result

        except Exception as exc:
            self.get_logger().error(f"Autonomous mission error: {exc}")
            goal_handle.abort()
            return result

        finally:
            self.mission = None
            with self.goal_lock:
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
