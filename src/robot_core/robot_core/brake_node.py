#!/usr/bin/env python3

"""
Brake node.

Controls the brake Arduino over serial.
"""

from dataclasses import dataclass
import time
from typing import Optional

import rclpy
from rclpy.node import Node
import serial

from robot_interfaces.msg import Brake, Malfunction
from robot_core.Tools import _get_param


@dataclass
class BrakeStatus:
    mode: str
    position: int
    max_position: int
    brake: str


class BrakeNode(Node):
    """
    Controls the brake firmware on Arduino.
    """

    EXPECTED_WHO = "BRAKE_UNO"
    CONTROL_PERIOD_S = 0.05
    PING_PERIOD_S = 0.1
    SERIAL_BOOT_WAIT_S = 2.0
    RECONNECT_DELAY_S = 2.0
    COMMAND_TIMEOUT_S = 1.0

    def __init__(self) -> None:
        super().__init__("brake_node", automatically_declare_parameters_from_overrides=True)

        self.serial_port = str(_get_param(self, "serial_port", "/dev/ttyUSB0"))
        self.baudrate = int(_get_param(self, "baudrate", 115200))
        self.serial_timeout_s = float(_get_param(self, "serial_timeout_s", 0.1))
        self.stroke_max = int(_get_param(self, "stroke_max", 250))
        self.topic_brake = str(_get_param(self, "topic_brake", "/robot/brake"))
        self.topic_malfunction = str(_get_param(self, "topic_malfunction", "/robot/malfunction"))

        self.desired_brake = True
        self.malfunction_active = False
        self.last_ping_time = 0.0
        self.last_connect_attempt_time = 0.0
        self.last_brake_command: Optional[bool] = None
        self.last_fault: Optional[str] = None
        self.serial_conn: Optional[serial.Serial] = None

        self.brake_subscription = self.create_subscription(
            Brake,
            self.topic_brake,
            self.brake_callback,
            10,
        )
        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD_S,
            self.control_loop,
        )
        self.malfunction_subs = self.create_subscription(
            Malfunction,
            self.topic_malfunction,
            self.malfunction_callback,
            10
        )

        self.ensure_serial()



    def brake_callback(self, msg: Brake) -> None:
        if self.malfunction_active:
            self.desired_brake = True
            return

        self.desired_brake = bool(msg.brake)
        if not self.desired_brake:
            self.last_ping_time = 0.0

    def control_loop(self) -> None:
        if not self.ensure_serial():
            return

        try:
            self.read_pending_lines()
            if self.malfunction_active:
                self.desired_brake = True
            self.sync_brake_state()
        except Exception as exc:
            self.get_logger().error(f"Brake control error: {exc}")
            self.close_serial()

    def malfunction_callback(self, msg: Malfunction) -> None:
        if msg.malfunction:
            self.malfunction_active = True
            self.desired_brake = True
            self.control_loop()

    def ensure_serial(self) -> bool:
        if self.serial_conn is not None and self.serial_conn.is_open:
            return True

        now = time.monotonic()
        if now - self.last_connect_attempt_time < self.RECONNECT_DELAY_S:
            return False

        self.last_connect_attempt_time = now

        try:
            self.open_serial()
            self.init_brake()
            self.last_ping_time = 0.0
            self.last_brake_command = None
            return True
        except Exception as exc:
            self.get_logger().error(
                f"Unable to initialize brake serial port {self.serial_port}: {exc}"
            )
            self.close_serial()
            return False

    def open_serial(self) -> None:
        self.serial_conn = serial.Serial(
            self.serial_port,
            self.baudrate,
            timeout=self.serial_timeout_s,
            write_timeout=self.serial_timeout_s,
        )

        # Arduino resets when the port is opened.
        time.sleep(self.SERIAL_BOOT_WAIT_S)

        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

    def init_brake(self) -> None:
        who = self.send_command("WHO")
        if who != self.EXPECTED_WHO:
            raise RuntimeError(
                f"Expected {self.EXPECTED_WHO} on {self.serial_port}, got {who!r}"
            )

        status = self.read_status()

        if status.max_position != self.stroke_max:
            response = self.send_command(f"SETMAX {self.stroke_max}")
            expected = f"MAX={self.stroke_max}"
            if response != expected:
                raise RuntimeError(
                    f"SETMAX response mismatch: expected {expected!r}, got {response!r}"
                )
            self.get_logger().info(f"Brake stroke_max set to {self.stroke_max}")

    def read_status(self) -> BrakeStatus:
        return self.parse_status(self.send_command("STATUS"))

    def parse_status(self, line: str) -> BrakeStatus:
        fields = {}
        for token in line.split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            fields[key] = value

        required_keys = {"MODE", "POSITION", "MAX", "BRAKE"}
        if not required_keys.issubset(fields):
            raise RuntimeError(f"Invalid STATUS line: {line!r}")

        return BrakeStatus(
            mode=fields["MODE"],
            position=int(fields["POSITION"]),
            max_position=int(fields["MAX"]),
            brake=fields["BRAKE"],
        )

    def sync_brake_state(self) -> None:
        if self.desired_brake:
            if self.last_brake_command is not True:
                response = self.send_command("BRAKE")
                if response != "BRAKING":
                    raise RuntimeError(f"Unexpected BRAKE response: {response!r}")
                self.last_brake_command = True
            return

        now = time.monotonic()
        if self.last_ping_time == 0.0 or now - self.last_ping_time >= self.PING_PERIOD_S:
            self.send_ping()
            self.last_ping_time = now

        self.last_brake_command = False

    def send_ping(self) -> None:
        self.write_line("PING")

    def send_command(self, command: str) -> str:
        self.write_line(command)

        deadline = time.monotonic() + self.COMMAND_TIMEOUT_S
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            if line.startswith("FAULT"):
                raise RuntimeError(f"Brake firmware fault after {command}: {line}")
            return line

        raise TimeoutError(f"Timeout waiting for response to {command}")

    def write_line(self, line: str) -> None:
        if self.serial_conn is None or not self.serial_conn.is_open:
            raise RuntimeError("Brake serial connection is not open")

        payload = f"{line}\n".encode("ascii")
        self.serial_conn.write(payload)
        self.serial_conn.flush()

    def read_line(self) -> Optional[str]:
        if self.serial_conn is None or not self.serial_conn.is_open:
            raise RuntimeError("Brake serial connection is not open")

        raw = self.serial_conn.readline()
        if not raw:
            return None

        line = raw.decode(errors="ignore").strip()
        if not line:
            return None

        return line

    def read_pending_lines(self) -> None:
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        while self.serial_conn.in_waiting > 0:
            line = self.read_line()
            if line is None:
                break
            self.handle_async_line(line)

    def handle_async_line(self, line: str) -> None:
        if not line.startswith("FAULT"):
            self.get_logger().warning(f"Unexpected brake line: {line}")
            return

        if line != self.last_fault:
            self.get_logger().error(f"Brake firmware fault: {line}")
            self.last_fault = line

    def close_serial(self) -> None:
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.serial_conn = None

    def destroy_node(self) -> bool:
        self.close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BrakeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
