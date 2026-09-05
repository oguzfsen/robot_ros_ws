#!/usr/bin/env python3

import time
from threading import Lock

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_interfaces.action import Autonomous
from robot_interfaces.msg import MissionState
from robot_core.Mission import Mission
from robot_core.Tools import _get_param


class AutoActNode(Node):

    def __init__(self) -> None:
        super().__init__(
            "auto_act_node",
            automatically_declare_parameters_from_overrides=True,
        )

        self.action_autonomous = str(
            _get_param(self, "action_autonomous", "/robot/autonomous")
        )
        self.topic_mission_state = str(
            _get_param(self, "topic_mission_state", "/robot/mission_state")
        )
        self.control_frequency = float(_get_param(self, "control_frequency", 10.0))

        self.mission: Mission | None = None
        self.goal_active = False
        self.shutdown_requested = False
        self.goal_lock = Lock()

        self.mission_state_publisher = self.create_publisher(
            MissionState,
            self.topic_mission_state,
            10,
        )

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

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.update,
        )

    def goal_callback(self, goal_request: Autonomous.Goal) -> GoalResponse:
        if not goal_request.mission_raw.strip():
            self.get_logger().warning("Empty autonomous mission rejected")
            return GoalResponse.REJECT

        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warning("An autonomous mission is already active")
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
                f"Autonomous mission started. mission_id={self.mission.cloud_id}"
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

    def update(self) -> None:
        if self.mission is None:
            return

        # TODO: Topic verilerini isle ve motion_node'a komut gonder.
        self.publish_mission_state()

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
