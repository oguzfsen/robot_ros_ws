#!/usr/bin/env python3

import codecs
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import serial

from robot_core.Tools import _get_param


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

        self.command_topic = str(_get_param(self, "command_topic", "/user/command"))
        self.mission_topic = str(_get_param(self, "mission_topic", "/user/mission"))
        self.rtcm_topic = str(_get_param(self, "rtcm_topic", "/station/rtcm"))
        self.status_topic = str(_get_param(self, "status_topic", "/robot/status"))

        self.command_pub = self.create_publisher(UInt8MultiArray, self.command_topic, 10)
        self.mission_pub = self.create_publisher(UInt8MultiArray, self.mission_topic, 10)

        self.serial_conn = None
        self._running = True
        self._json_decoder = json.JSONDecoder()
        self._utf8_decoder = codecs.getincrementaldecoder("utf-8")()
        self._text_buffer = ""
        self._last_connect_log_time = 0.0

        self._read_thread = threading.Thread(
            target=self._read_serial_loop,
            daemon=True,
        )
        self._read_thread.start()

        self.get_logger().info("telemetry_node up and running...")

    def _connect_serial(self) -> bool:
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
                chunk = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                if not chunk:
                    continue

                self._text_buffer += self._utf8_decoder.decode(chunk)
                self._process_complete_json_messages()
            except UnicodeDecodeError:
                self.get_logger().warn(
                    "Telemetry hatti UTF-8 disi veri gonderdi; bu adimda veri yok sayildi."
                )
                self._reset_read_buffer()
            except Exception as exc:
                if not self._running:
                    break
                self.get_logger().warn(f"Telemetry serial okuma hatasi: {exc}")
                self._close_serial()
                time.sleep(self.reconnect_delay_s)

    def _process_complete_json_messages(self) -> None:
        while True:
            stripped_buffer = self._text_buffer.lstrip()
            if not stripped_buffer:
                self._text_buffer = ""
                return

            if stripped_buffer != self._text_buffer:
                self._text_buffer = stripped_buffer

            try:
                payload_obj, end_index = self._json_decoder.raw_decode(self._text_buffer)
            except json.JSONDecodeError:
                if self._text_buffer[0] not in "{[":
                    self._drop_until_json_start()
                    continue
                return

            json_text = self._text_buffer[:end_index]
            self._text_buffer = self._text_buffer[end_index:]
            self._handle_json_message(payload_obj, json_text)

    def _handle_json_message(self, payload_obj, json_text: str) -> None:
        self.get_logger().info(f"Telemetry RX: {json_text}")

        if not isinstance(payload_obj, dict):
            self.get_logger().warn("Telemetry mesaji JSON ama obje degil; publish edilmedi.")
            return

        kind = self._detect_message_kind(payload_obj)
        if kind == "command":
            self._publish_bytes(self.command_pub, json_text.encode("utf-8"))
            return

        if kind == "mission":
            self._publish_bytes(self.mission_pub, json_text.encode("utf-8"))
            return

        self.get_logger().warn(
            "Telemetry JSON mesaji command/mission olarak siniflandirilamadi."
        )

    def _detect_message_kind(self, payload_obj: dict) -> str:
        if "command_type" in payload_obj:
            return "command"

        if "payload" in payload_obj and "request_id" in payload_obj:
            return "mission"

        return "unknown"

    def _publish_bytes(self, publisher, payload: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = list(payload)
        publisher.publish(msg)

    def _drop_until_json_start(self) -> None:
        start_indexes = [
            index
            for index in (
                self._text_buffer.find("{"),
                self._text_buffer.find("["),
            )
            if index >= 0
        ]

        if not start_indexes:
            self._text_buffer = ""
            return

        self._text_buffer = self._text_buffer[min(start_indexes):]

    def _reset_read_buffer(self) -> None:
        self._text_buffer = ""
        self._utf8_decoder = codecs.getincrementaldecoder("utf-8")()

    def _close_serial(self) -> None:
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
