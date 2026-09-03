#!/usr/bin/env python3

import base64
import json
import threading
import time
from datetime import datetime, timezone
from typing import Any

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Command, Gnss, MissionState, RobotStatus
from robot_interfaces.srv import MissionSet
from std_msgs.msg import Int32
from std_msgs.msg import UInt8MultiArray

from robot_core.Mission import Mission
from robot_core.Tools import RobotState, _get_param, _wait_for_service
from robot_core.signalr_client import SignalRLongPollingClient


class SignalRNode(Node):
    def __init__(self):
        super().__init__(
            "signalr_node",
            automatically_declare_parameters_from_overrides=True,
        )

        self.hub_url = str(
            _get_param(self, "hub_url", "https://gorev-yonetici.com/robotHub")
        )
        self.send_method_name = str(_get_param(self, "send_method_name", "SendMessage"))
        self.receive_method_name = str(
            _get_param(self, "receive_method_name", "ReceiveMessage")
        )
        self.reconnect_delay_s = float(_get_param(self, "reconnect_delay_s", 1.0))
        self.poll_timeout_s = float(_get_param(self, "poll_timeout_s", 15.0))
        self.http_timeout_s = float(_get_param(self, "http_timeout_s", 15.0))
        self.status_period_s = float(_get_param(self, "status_period_s", 1.0))
        self.robot_id = str(_get_param(self, "robot_id", "UTAEMBOT"))

        self.topic_state = str(_get_param(self, "topic_state", "/robot/state"))
        self.topic_gnss = str(_get_param(self, "topic_gnss", "/robot/gnss"))
        self.topic_rtcm = str(_get_param(self, "topic_rtcm", "/station/rtcm"))
        self.topic_command = str(_get_param(self, "topic_command", "/user/command"))
        self.topic_mission = str(_get_param(self, "topic_mission", "/user/mission"))
        self.topic_mission_state = str(_get_param(self, "topic_mission_state", "/robot/mission_state"))
        self.topic_status = str(_get_param(self, "topic_status", "/robot/status"))
        self.topic_mission_set = str(_get_param(self, "topic_mission_set", "/database/mission_set"))

        self.command_pub = self.create_publisher(Command, self.topic_command, 10)
        self.rtcm_pub = self.create_publisher(UInt8MultiArray, self.topic_rtcm, 10)
        self.status_pub = self.create_publisher(RobotStatus, self.topic_status, 10)
        self.state_sub = self.create_subscription(Int32, self.topic_state, self.state_callback, 10)
        self.gnss_sub = self.create_subscription(Gnss, self.topic_gnss, self.gnss_callback, 10)
        self.mission_state_sub = self.create_subscription(MissionState, self.topic_mission_state, self.mission_state_callback, 10)

        self.mission_set_client = self.create_client(MissionSet, self.topic_mission_set)
        _wait_for_service(self, self.mission_set_client)

        self._state_lock = threading.Lock()
        self._pending_status_responses: list[dict[str, Any]] = []
        self._latest_robot_state = int(RobotState.IDLE)
        self._latest_gnss: Gnss | None = None
        self._latest_mission_state: MissionState | None = None
        self._rtcm_frame_count = 0
        self._last_status_log_time = 0.0
        self._last_status_error_log_time = 0.0
        

        self.signalr_client = SignalRLongPollingClient(
            hub_url=self.hub_url,
            send_method_name=self.send_method_name,
            receive_method_name=self.receive_method_name,
            reconnect_delay_s=self.reconnect_delay_s,
            poll_timeout_s=self.poll_timeout_s,
            http_timeout_s=self.http_timeout_s,
            on_receive_message=self.handle_signalr_message,
            log_info=self.get_logger().info,
            log_warn=self.get_logger().warn,
        )
        self.signalr_client.start()

        self.status_timer = self.create_timer(self.status_period_s, self.publish_status)


    def handle_signalr_message(self, signalr_topic: str, json_text: str) -> None:
        if signalr_topic == self.topic_command:
            self._handle_command_signalr_message(json_text)
            return

        if signalr_topic == self.topic_mission:
            self._handle_mission_signalr_message(json_text)
            return

        if signalr_topic == self.topic_rtcm:
            self._handle_rtcm_signalr_message(json_text)
            return


    def _handle_command_signalr_message(self, json_text: str) -> None:
        payload_obj = self._parse_json_object(json_text, self.topic_command)
        if payload_obj is None:
            return

        if not self._is_targeted_to_this_robot(payload_obj, allow_broadcast=False):
            return

        self._handle_command_message(payload_obj)

    def _handle_mission_signalr_message(self, json_text: str) -> None:
        payload_obj = self._parse_json_object(json_text, self.topic_mission)
        if payload_obj is None:
            return

        if not self._is_targeted_to_this_robot(payload_obj, allow_broadcast=False):
            return

        self._handle_mission_message(payload_obj)

    def _handle_rtcm_signalr_message(self, json_text: str) -> None:
        payload_obj = self._parse_json_object(json_text, self.topic_rtcm)
        if payload_obj is None:
            return

        if not self._is_targeted_to_this_robot(payload_obj, allow_broadcast=True):
            return

        payload_base64 = str(payload_obj.get("payload_base64", "")).strip()
        if not payload_base64:
            self.get_logger().warn("SignalR RTCM mesaji payload_base64 icermiyor.")
            return

        try:
            frame = base64.b64decode(payload_base64, validate=True)
        except Exception as exc:
            self.get_logger().warn(f"SignalR RTCM base64 decode hatasi: {exc}")
            return

        if not frame:
            self.get_logger().warn("SignalR RTCM mesaji bos payload ile geldi.")
            return

        byte_count = self._coerce_int(payload_obj.get("byte_count"), default_value=0)
        if byte_count > 0 and byte_count != len(frame):
            self.get_logger().warn(
                "SignalR RTCM byte_count uyusmuyor: "
                f"beklenen={byte_count} gercek={len(frame)}"
            )
            return

        self._publish_bytes(self.rtcm_pub, frame)
        self._rtcm_frame_count += 1


    def _handle_command_message(self, payload_obj: dict[str, Any]) -> None:
        request_id = str(payload_obj.get("request_id", "")).strip()
        command_type = str(payload_obj.get("command_type", "")).strip()
        payload = self._normalize_json_payload(payload_obj.get("payload"))
        source = str(payload_obj.get("source", "")).strip()

        self.get_logger().info(
            "Command SignalR'dan alindi ve ROS'a basiliyor: "
            f"request_id={request_id or 'missing'} "
            f"command_type={command_type or 'missing'}"
        )

        command_msg = Command()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.robot_id = self.robot_id
        command_msg.request_id = request_id
        command_msg.command_type = command_type
        command_msg.payload = payload
        command_msg.source = source
        self.command_pub.publish(command_msg)

        self._queue_status_response(
            request_id=request_id,
            success=True,
            error_text="",
            response_text="command received by signalr_node",
        )

    def _handle_mission_message(self, payload_obj: dict[str, Any]) -> None:
        request_id = str(payload_obj.get("request_id", "")).strip()

        try:
            receive_date = (
                str(payload_obj.get("receive_date", "")).strip() or self._utc_now_iso()
            )
            mission_json = self._extract_mission_json(payload_obj.get("payload"))
            mission = Mission(mission_json, last_waypoint=0)
            self._store_mission_via_database_service(
                mission_id=mission.cloud_id,
                receive_date=receive_date,
                raw_json=mission_json,
            )

            self._queue_status_response(
                request_id=request_id,
                success=True,
                error_text="",
                response_text=f"mission_id={mission.cloud_id} veritabanina kaydedildi",
            )
            self.get_logger().info(
                f"SignalR mission kaydedildi: mission_id={mission.cloud_id}"
            )
        except Exception as exc:
            self._queue_status_response(
                request_id=request_id,
                success=False,
                error_text=str(exc),
                response_text="",
            )
            self.get_logger().error(f"SignalR mission kaydi basarisiz: {exc}")

    def state_callback(self, msg: Int32) -> None:
        self._latest_robot_state = int(msg.data)

    def gnss_callback(self, msg: Gnss) -> None:
        self._latest_gnss = msg

    def mission_state_callback(self, msg: MissionState) -> None:
        self._latest_mission_state = msg

    def publish_status(self) -> None:
        pending_responses = self._drain_pending_status_responses()

        try:
            status_msg = self._build_robot_status_message(pending_responses)
            self.status_pub.publish(status_msg)
            status_payload = self._robot_status_to_dict(status_msg)
            json_text = json.dumps(
                status_payload,
                ensure_ascii=True,
                separators=(",", ":"),
            )
            self.signalr_client.send_now(
                self.topic_status,
                json_text                
            )

            now = time.monotonic()
            if pending_responses or now - self._last_status_log_time >= 5.0:
                self._last_status_log_time = now
                #self.get_logger().info(
                #    "Robot status SignalR'a gonderildi: "
                #    f"state={status_payload['robot_state']} "
                #    f"mission_id={status_payload['mission_id']} "
                #    f"responses={len(pending_responses)}"
                #)
        except Exception as exc:
            if pending_responses:
                self._restore_pending_status_responses(pending_responses)

            now = time.monotonic()
            if now - self._last_status_error_log_time >= 5.0:
                self._last_status_error_log_time = now
                self.get_logger().warn(f"SignalR status gonderimi basarisiz: {exc}")

    def _extract_mission_json(self, payload: Any) -> str:
        if isinstance(payload, dict):
            return json.dumps(payload, ensure_ascii=False)

        if isinstance(payload, str):
            raw_json = payload.strip()
            if not raw_json:
                raise ValueError("mission payload is empty")

            parsed = json.loads(raw_json)
            if not isinstance(parsed, dict):
                raise ValueError("mission payload must decode to a JSON object")
            return json.dumps(parsed, ensure_ascii=False)

        raise ValueError("mission payload must be a JSON string or object")

    def _normalize_json_payload(self, payload: Any) -> str:
        if payload is None:
            return ""

        if isinstance(payload, str):
            return payload

        return json.dumps(payload, ensure_ascii=False)

    def _store_mission_via_database_service(self, mission_id: int, receive_date: str, raw_json: str,) -> None:
        request = MissionSet.Request()
        request.mission_id = int(mission_id)
        request.receive_date = receive_date
        request.raw_json = raw_json

        future = self.mission_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            raise RuntimeError("database mission_set timed out")

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"database mission_set exception: {exception}")

        response = future.result()
        if response is None:
            raise RuntimeError("database mission_set returned no response")

        if not response.success:
            raise RuntimeError(response.error or "database mission_set failed")


    def _queue_status_response(
        self,
        request_id: str,
        success: bool,
        error_text: str,
        response_text: str,
    ) -> None:
        response_entry = {
            "request_id": request_id,
            "success": bool(success),
            "error_text": error_text,
            "response_text": response_text,
        }

        with self._state_lock:
            self._pending_status_responses.append(response_entry)

    def _drain_pending_status_responses(self) -> list[dict[str, Any]]:
        with self._state_lock:
            responses = list(self._pending_status_responses)
            self._pending_status_responses.clear()
            return responses

    def _restore_pending_status_responses(
        self,
        responses: list[dict[str, Any]],
    ) -> None:
        with self._state_lock:
            self._pending_status_responses = responses + self._pending_status_responses

    def _build_robot_status_message(
        self,
        pending_responses: list[dict[str, Any]],
    ) -> RobotStatus:
        gnss = self._latest_gnss
        mission_state = self._latest_mission_state

        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.robot_state = int(self._latest_robot_state)
        msg.latitude = 0.0 if gnss is None else float(gnss.latitude)
        msg.longitude = 0.0 if gnss is None else float(gnss.longitude)
        msg.altitude = 0.0 if gnss is None else float(gnss.altitude)
        msg.speed = 0.0 if gnss is None else float(gnss.ground_speed_m_s)
        msg.heading = 0.0 if gnss is None else float(gnss.heading_deg)
        msg.gnss_fix_type = "" if gnss is None else str(gnss.fix_type)
        msg.accuracy = 0.0 if gnss is None else float(gnss.accuracy_m)
        msg.energy = 0
        msg.tank = 100
        msg.malfunction = False
        msg.emergency = False
        msg.mission_id = 0 if mission_state is None else int(mission_state.mission_id)
        msg.completion_rate = (
            0 if mission_state is None else int(mission_state.completion_rate)
        )
        msg.responses = json.dumps(pending_responses, ensure_ascii=False)
        return msg

    def _robot_status_to_dict(self, msg: RobotStatus) -> dict[str, Any]:
        return {
            "robot_id": getattr(msg, "robot_id", ""),
            "robot_state": int(msg.robot_state),
            "latitude": round(float(msg.latitude), 7),
            "longitude": round(float(msg.longitude), 7),
            "altitude": round(float(msg.altitude), 3),
            "speed": round(float(msg.speed), 3),
            "heading": round(float(msg.heading), 3),
            "gnss_fix_type": str(msg.gnss_fix_type),
            "accuracy": round(float(msg.accuracy), 3),
            "energy": int(msg.energy),
            "tank": int(msg.tank),
            "malfunction": bool(msg.malfunction),
            "emergency": bool(msg.emergency),
            "mission_id": int(msg.mission_id),
            "completion_rate": int(msg.completion_rate),
            "responses": str(msg.responses or ""),
        }

    def _publish_bytes(self, publisher, payload: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = list(payload)
        publisher.publish(msg)

    def _is_targeted_to_this_robot(
        self,
        payload_obj: dict[str, Any],
        allow_broadcast: bool,
    ) -> bool:
        target_robot_id = str(payload_obj.get("robot_id", "")).strip()

        if not target_robot_id:
            if allow_broadcast:
                return True

            request_id = str(payload_obj.get("request_id", "")).strip()
            self.get_logger().warn(
                "SignalR mesaji robot_id icermiyor; yok sayildi: "
                f"request_id={request_id or 'missing'}"
            )
            return False

        return target_robot_id == self.robot_id

    def _parse_json_object(
        self,
        json_text: str,
        topic_name: str,
    ) -> dict[str, Any] | None:
        try:
            payload_obj = json.loads(json_text)
        except Exception as exc:
            self.get_logger().warn(
                f"SignalR JSON cozumleme hatasi topic={topic_name}: {exc}"
            )
            return None

        if not isinstance(payload_obj, dict):
            self.get_logger().warn(
                f"SignalR mesaji obje degil topic={topic_name}; yok sayildi."
            )
            return None

        return payload_obj


    @staticmethod
    def _utc_now_iso() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
            "+00:00",
            "Z",
        )

    @staticmethod
    def _coerce_int(raw_value: Any, default_value: int = 0) -> int:
        try:
            if raw_value is None or raw_value == "":
                return default_value
            return int(raw_value)
        except Exception:
            return default_value

    def destroy_node(self):
        try:
            self.signalr_client.stop()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SignalRNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
