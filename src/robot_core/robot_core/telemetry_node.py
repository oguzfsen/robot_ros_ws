#!/usr/bin/env python3

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

import serial

from robot_core.Mission import Mission
from robot_core.Tools import RobotState, _get_param


class TelemetryNode(Node):
    def __init__(self):
        super().__init__(
            "telemetry_node",
            automatically_declare_parameters_from_overrides=True,
        )

        default_serial_port = (
            "/dev/serial/by-id/"
            "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_"
            "0001-if00-port0"
        )

        self.serial_port = str(_get_param(self, "serial_port", default_serial_port))
        self.baudrate = int(_get_param(self, "baudrate", 460800))
        self.serial_timeout_s = float(_get_param(self, "serial_timeout_s", 0.1))
        self.reconnect_delay_s = float(_get_param(self, "reconnect_delay_s", 2.0))
        self.robot_id = str(_get_param(self, "robot_id", "UTAEMBOT"))

        self.state_topic = str(_get_param(self, "state_topic", "/robot/state"))
        self.gnss_topic = str(_get_param(self, "gnss_topic", "/robot/gnss"))
        self.mission_state_topic = str(
            _get_param(self, "mission_state_topic", "/robot/mission_state")
        )
        self.command_topic = str(_get_param(self, "command_topic", "/user/command"))
        self.rtcm_topic = str(_get_param(self, "rtcm_topic", "/station/rtcm"))
        self.status_topic = str(_get_param(self, "status_topic", "/robot/status"))
        self.database_mission_set_service = str(
            _get_param(self, "database_mission_set_service", "/database/mission_set")
        )

        self.command_pub = self.create_publisher(Command, self.command_topic, 10)
        self.rtcm_pub = self.create_publisher(UInt8MultiArray, self.rtcm_topic, 10)
        self.status_pub = self.create_publisher(RobotStatus, self.status_topic, 10)
        self.state_sub = self.create_subscription(
            Int32,
            self.state_topic,
            self.state_callback,
            10,
        )
        self.gnss_sub = self.create_subscription(
            Gnss,
            self.gnss_topic,
            self.gnss_callback,
            10,
        )
        self.mission_state_sub = self.create_subscription(
            MissionState,
            self.mission_state_topic,
            self.mission_state_callback,
            10,
        )
        self.mission_set_client = self.create_client(
            MissionSet,
            self.database_mission_set_service,
        )

        self.serial_conn = None
        self._running = True
        self._rx_buffer = bytearray()
        self._last_connect_log_time = 0.0
        self._serial_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._pending_status_responses: list[dict[str, Any]] = []
        self._latest_robot_state = int(RobotState.IDLE)
        self._latest_gnss: Gnss | None = None
        self._latest_mission_state: MissionState | None = None

        self.status_timer = self.create_timer(1.0, self.publish_status)

        self._read_thread = threading.Thread(
            target=self._read_serial_loop,
            daemon=True,
        )
        self._read_thread.start()

        self.get_logger().info("telemetry_node up and running...")

    def _connect_serial(self) -> bool:
        with self._serial_lock:
            if self.serial_conn is not None and self.serial_conn.is_open:
                return True

            try:
                self.serial_conn = serial.Serial(
                    self.serial_port,
                    self.baudrate,
                    timeout=self.serial_timeout_s,
                )
                self._reset_read_buffer()
                self.get_logger().info(
                    f"Telemetry serial baglandi: {self.serial_port} @ {self.baudrate}"
                )
                return True
            except Exception as exc:
                now = time.monotonic()
                if now - self._last_connect_log_time >= self.reconnect_delay_s:
                    self._last_connect_log_time = now
                    self.get_logger().warn(
                        f"Telemetry serial baglanamadi: {self.serial_port} ({exc})"
                    )
                self.serial_conn = None
                return False

    def _read_serial_loop(self) -> None:
        while self._running:
            if not self._connect_serial():
                time.sleep(self.reconnect_delay_s)
                continue

            try:
                with self._serial_lock:
                    if self.serial_conn is None:
                        continue
                    chunk = self.serial_conn.read(self.serial_conn.in_waiting or 1)

                if not chunk:
                    continue

                self._rx_buffer.extend(chunk)
                self._process_serial_buffer()
            except Exception as exc:
                if not self._running:
                    break
                self.get_logger().warn(f"Telemetry serial okuma hatasi: {exc}")
                self._close_serial()
                time.sleep(self.reconnect_delay_s)

    def _process_serial_buffer(self) -> None:
        while True:
            self._trim_leading_whitespace()
            if not self._rx_buffer:
                return

            first_byte = self._rx_buffer[0]

            if first_byte == 0xD3:
                rtcm_frame = self._extract_rtcm_frame()
                if rtcm_frame is None:
                    return
                self._publish_bytes(self.rtcm_pub, rtcm_frame)
                continue

            if first_byte in (ord("{"), ord("[")):
                json_end = self._find_complete_json_end(self._rx_buffer)
                if json_end is None:
                    return

                json_bytes = bytes(self._rx_buffer[:json_end])
                del self._rx_buffer[:json_end]

                try:
                    json_text = json_bytes.decode("utf-8")
                    payload_obj = json.loads(json_text)
                except Exception as exc:
                    self.get_logger().warn(f"Telemetry JSON cozumleme hatasi: {exc}")
                    continue

                self._handle_json_message(payload_obj, json_text)
                continue

            self._drop_until_known_start()
            if self._rx_buffer and self._rx_buffer[0] == first_byte:
                del self._rx_buffer[0]

    def _handle_json_message(self, payload_obj, json_text: str) -> None:
        self.get_logger().info(f"Telemetry RX: {json_text}")

        if not isinstance(payload_obj, dict):
            self.get_logger().warn("Telemetry mesaji JSON ama obje degil; yok sayildi.")
            return

        kind = self._detect_message_kind(payload_obj)
        if kind == "command":
            self._handle_command_message(payload_obj)
            return

        if kind == "mission":
            self._handle_mission_message(payload_obj)
            return

        self.get_logger().warn(
            "Telemetry JSON mesaji command/mission olarak siniflandirilamadi."
        )

    def _handle_command_message(self, payload_obj: dict[str, Any]) -> None:
        request_id = str(payload_obj.get("request_id", "")).strip()
        command_type = str(payload_obj.get("command_type", "")).strip()
        payload = self._normalize_json_payload(payload_obj.get("payload"))
        source = str(payload_obj.get("source", "")).strip()
        robot_id = str(payload_obj.get("robot_id", "")).strip() or self.robot_id

        command_msg = Command()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.robot_id = robot_id
        command_msg.request_id = request_id
        command_msg.command_type = command_type
        command_msg.payload = payload
        command_msg.source = source
        self.command_pub.publish(command_msg)

        self._queue_status_response(
            request_id=request_id,
            success=True,
            error_text="",
            response_text="command received by telemetry_node",
        )

    def _handle_mission_message(self, payload_obj: dict[str, Any]) -> None:
        request_id = str(payload_obj.get("request_id", "")).strip()

        try:
            receive_date = str(payload_obj.get("receive_date", "")).strip() or self._utc_now_iso()
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
                f"Telemetry mission kaydedildi: mission_id={mission.cloud_id}"
            )
        except Exception as exc:
            self._queue_status_response(
                request_id=request_id,
                success=False,
                error_text=str(exc),
                response_text="",
            )
            self.get_logger().error(f"Telemetry mission kaydi basarisiz: {exc}")

    def _detect_message_kind(self, payload_obj: dict) -> str:
        if "command_type" in payload_obj:
            return "command"

        if "payload" in payload_obj and "request_id" in payload_obj:
            return "mission"

        return "unknown"

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
            self._write_serial_json(status_payload)
        except Exception as exc:
            if pending_responses:
                self._restore_pending_status_responses(pending_responses)
            self.get_logger().warn(f"Telemetry status yazimi basarisiz: {exc}")

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

    def _store_mission_via_database_service(
        self,
        mission_id: int,
        receive_date: str,
        raw_json: str,
    ) -> None:
        if not self._wait_for_service(self.mission_set_client):
            raise RuntimeError(
                f"database mission service unavailable: {self.database_mission_set_service}"
            )

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

    def _parse_status_responses(self, responses_text: str) -> list[dict[str, Any]]:
        raw_text = str(responses_text or "").strip()
        if not raw_text:
            return []

        try:
            parsed = json.loads(raw_text)
        except json.JSONDecodeError:
            self.get_logger().warn("RobotStatus.responses gecerli JSON degil; sifirlandi.")
            return []

        if isinstance(parsed, list):
            return parsed

        self.get_logger().warn("RobotStatus.responses JSON list degil; sifirlandi.")
        return []

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
            "latitude": float(msg.latitude),
            "longitude": float(msg.longitude),
            "altitude": float(msg.altitude),
            "speed": float(msg.speed),
            "heading": float(msg.heading),
            "gnss_fix_type": str(msg.gnss_fix_type),
            "accuracy": float(msg.accuracy),
            "energy": int(msg.energy),
            "tank": int(msg.tank),
            "malfunction": bool(msg.malfunction),
            "emergency": bool(msg.emergency),
            "mission_id": int(msg.mission_id),
            "completion_rate": int(msg.completion_rate),
            "responses": str(msg.responses or ""),
        }

    def _write_serial_json(self, payload_obj: dict[str, Any]) -> None:
        payload = (json.dumps(payload_obj, ensure_ascii=False) + "\n").encode("utf-8")
        self._write_serial_bytes(payload)

    def _write_serial_bytes(self, payload: bytes) -> None:
        if not self._connect_serial():
            raise RuntimeError("serial hatti bagli degil")

        with self._serial_lock:
            if self.serial_conn is None:
                raise RuntimeError("serial connection is not available")
            self.serial_conn.write(payload)

    def _publish_bytes(self, publisher, payload: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = list(payload)
        publisher.publish(msg)

    def _extract_rtcm_frame(self) -> bytes | None:
        if len(self._rx_buffer) < 3:
            return None

        payload_length = ((self._rx_buffer[1] & 0x03) << 8) | self._rx_buffer[2]
        frame_length = 3 + payload_length + 3
        if len(self._rx_buffer) < frame_length:
            return None

        frame = bytes(self._rx_buffer[:frame_length])
        del self._rx_buffer[:frame_length]
        return frame

    def _find_complete_json_end(self, buffer: bytearray) -> int | None:
        depth = 0
        in_string = False
        escape = False

        for index, byte in enumerate(buffer):
            if in_string:
                if escape:
                    escape = False
                    continue
                if byte == ord("\\"):
                    escape = True
                    continue
                if byte == ord('"'):
                    in_string = False
                continue

            if byte == ord('"'):
                in_string = True
                continue

            if byte in (ord("{"), ord("[")):
                depth += 1
                continue

            if byte in (ord("}"), ord("]")):
                depth -= 1
                if depth == 0:
                    return index + 1

        return None

    def _drop_until_known_start(self) -> None:
        start_indexes = [
            index
            for index in (
                self._rx_buffer.find(b"{"),
                self._rx_buffer.find(b"["),
                self._rx_buffer.find(b"\xD3"),
            )
            if index >= 0
        ]

        if not start_indexes:
            self._rx_buffer.clear()
            return

        del self._rx_buffer[:min(start_indexes)]

    def _trim_leading_whitespace(self) -> None:
        while self._rx_buffer and chr(self._rx_buffer[0]).isspace():
            del self._rx_buffer[0]

    def _reset_read_buffer(self) -> None:
        self._rx_buffer.clear()

    def _wait_for_service(self, client, timeout_sec: float = 1.0) -> bool:
        start_time = time.monotonic()
        while rclpy.ok() and not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Waiting for service: {self.database_mission_set_service}"
            )
            if time.monotonic() - start_time >= timeout_sec:
                return False
        return client.service_is_ready()

    @staticmethod
    def _utc_now_iso() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
            "+00:00",
            "Z",
        )

    def _close_serial(self) -> None:
        with self._serial_lock:
            try:
                if self.serial_conn is not None and self.serial_conn.is_open:
                    self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    def destroy_node(self):
        self._running = False
        self._close_serial()

        try:
            if self._read_thread.is_alive():
                self._read_thread.join(timeout=1.0)
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
