#!/usr/bin/env python3

"""
GNSS node.

Reads high precision position from a u-blox based HERE4 receiver over USB,
forwards RTCM correction bytes received on a ROS topic to the receiver, and
publishes the resulting absolute position.
"""

import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from serial import Serial, SerialException
from std_msgs.msg import UInt8MultiArray

from robot_interfaces.msg import GnssAbs


class GnssNode(Node):
    """Bridge a serial u-blox GNSS receiver into ROS topics."""

    NAV_PVT_PAYLOAD_LENGTH = 92
    NAV_HPPOSLLH_PAYLOAD_LENGTH = 36

    def __init__(self):
        """Initialize parameters, serial link, publisher, and subscriber."""
        super().__init__("gnss_node", automatically_declare_parameters_from_overrides=True)

        self.serial_port = str(self._get_param("serial_port", "/dev/ttyACM0"))
        self.serial_baud = int(self._get_param("serial_baud", 9600))
        self.serial_timeout_sec = float(self._get_param("serial_timeout_sec", 0.2))
        self.publish_hz = float(self._get_param("publish_hz", 1.0))
        self.poll_timeout_sec = float(self._get_param("poll_timeout_sec", 0.6))
        self.correction_freshness_sec = float(self._get_param("correction_freshness_sec", 3.0))
        self.frame_id = str(self._get_param("frame_id", "gnss"))

        self.serial_handle: Optional[Serial] = None
        self.serial_lock = threading.Lock()
        self.last_correction_rx_time: Optional[float] = None
        self.last_open_error_log_time = 0.0

        self.publisher = self.create_publisher(GnssAbs, "/robot/gnssAbs", 10)
        self.correction_sub = self.create_subscription(
            UInt8MultiArray,
            "/robot/gnssfix",
            self.correction_callback,
            50,
        )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)

        self._ensure_serial_open()
        self.get_logger().info(
            f"gnss_node started on {self.serial_port} at {self.serial_baud} baud"
        )

    def _get_param(self, name: str, default_value=None):
        """
        Generic ROS2 parameter reader.

        Returns plain Python values and handles Humble edge cases.
        """
        try:
            if self.has_parameter(name):
                param = self.get_parameter(name)
            else:
                param = self.declare_parameter(name, default_value)
        except Exception:
            param = self.get_parameter(name)

        if not isinstance(param, Parameter):
            return param

        try:
            return param.value
        except Exception:
            pass

        try:
            value = param.get_parameter_value()

            if value.integer_value != 0 or isinstance(default_value, int):
                return value.integer_value
            if value.double_value != 0.0 or isinstance(default_value, float):
                return value.double_value
            if value.bool_value in (True, False):
                return value.bool_value
            if value.string_value != "":
                return value.string_value
            if value.integer_array_value:
                return list(value.integer_array_value)
            if value.double_array_value:
                return list(value.double_array_value)
            if value.bool_array_value:
                return list(value.bool_array_value)
            if value.string_array_value:
                return list(value.string_array_value)
        except Exception:
            pass

        return default_value

    def _ensure_serial_open(self) -> bool:
        """Open the configured serial port if it is not already open."""
        if self.serial_handle is not None and self.serial_handle.is_open:
            return True

        try:
            self.serial_handle = Serial(
                port=self.serial_port,
                baudrate=self.serial_baud,
                timeout=self.serial_timeout_sec,
            )
            self.get_logger().info(f"Connected to GNSS receiver on {self.serial_port}")
            return True
        except SerialException as ex:
            now = time.monotonic()

            if now - self.last_open_error_log_time >= 5.0:
                self.get_logger().error(f"Failed to open GNSS serial port: {ex}")
                self.last_open_error_log_time = now

            self.serial_handle = None
            return False

    def correction_callback(self, msg: UInt8MultiArray) -> None:
        """Forward incoming RTCM bytes to the GNSS receiver over USB."""
        if not msg.data:
            return

        if not self._ensure_serial_open():
            return

        try:
            with self.serial_lock:
                self.serial_handle.write(bytes(msg.data))
                self.serial_handle.flush()
            self.last_correction_rx_time = time.monotonic()
        except SerialException as ex:
            self.get_logger().error(f"Failed to forward RTCM correction bytes: {ex}")
            self._close_serial()

    def timer_callback(self) -> None:
        """Poll the receiver and publish one GNSS sample."""
        if not self._ensure_serial_open():
            self.publish_invalid()
            return

        try:
            poll_buffer = self.poll_receiver()
            pvt = self.parse_nav_pvt(poll_buffer)
            hppos = self.parse_nav_hpposllh(poll_buffer)
            self.publish_solution(pvt, hppos)
        except Exception as ex:
            self.get_logger().error(f"GNSS poll error: {ex}")
            self.publish_invalid()

    def poll_receiver(self) -> bytes:
        """Request NAV-PVT and NAV-HPPOSLLH frames from the receiver."""
        poll_messages = (
            self.build_poll_message(0x01, 0x07),
            self.build_poll_message(0x01, 0x14),
        )
        deadline = time.monotonic() + self.poll_timeout_sec
        buffer = bytearray()

        with self.serial_lock:
            self.serial_handle.reset_input_buffer()

            for message in poll_messages:
                self.serial_handle.write(message)

            self.serial_handle.flush()

            while time.monotonic() < deadline:
                chunk = self.serial_handle.read(self.serial_handle.in_waiting or 256)

                if chunk:
                    buffer.extend(chunk)

                    if (
                        self.extract_ubx_packet(buffer, 0x01, 0x07) is not None
                        and self.extract_ubx_packet(buffer, 0x01, 0x14) is not None
                    ):
                        break

        return bytes(buffer)

    def publish_solution(self, pvt: Optional[dict], hppos: Optional[dict]) -> None:
        """Build and publish the outgoing ROS GNSS message."""
        msg = GnssAbs()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        position = hppos if hppos is not None and not hppos["invalid"] else pvt

        if position is None:
            msg.valid = False
            self.publisher.publish(msg)
            return

        fix_ok = bool(pvt is not None and pvt["fix_ok"])
        carr_soln = int(pvt["carr_soln"]) if pvt is not None else 0
        diff_soln = bool(pvt is not None and pvt["diff_soln"])
        recent_rtcm = self.has_recent_correction()

        msg.latitude = float(position["latitude"])
        msg.longitude = float(position["longitude"])
        msg.altitude_m = float(position["altitude_m"])
        msg.horizontal_accuracy_m = float(position["horizontal_accuracy_m"])
        msg.vertical_accuracy_m = float(position["vertical_accuracy_m"])
        msg.satellites = int(pvt["satellites"]) if pvt is not None else 0
        msg.valid = fix_ok
        msg.corrected = bool(recent_rtcm and diff_soln)
        msg.fixed = bool(recent_rtcm and carr_soln == 2)

        self.publisher.publish(msg)
        self.log_solution_summary(msg, pvt, recent_rtcm)

    def log_solution_summary(
        self,
        msg: GnssAbs,
        pvt: Optional[dict],
        recent_rtcm: bool,
    ) -> None:
        """Log a concise GNSS solution summary instead of the full message dump."""
        fix_type = int(pvt["fix_type"]) if pvt is not None else 0
        carr_soln = int(pvt["carr_soln"]) if pvt is not None else 0
        diff_soln = bool(pvt["diff_soln"]) if pvt is not None else False

        self.get_logger().info(
            "GNSS: valid=%s corrected=%s fixed=%s carrier=%s fix_type=%s "
            "diff=%s recent_rtcm=%s sats_used=%d h_acc=%.3fm v_acc=%.3fm"
            % (
                str(msg.valid).lower(),
                str(msg.corrected).lower(),
                str(msg.fixed).lower(),
                self.describe_carrier_solution(carr_soln),
                self.describe_fix_type(fix_type),
                str(diff_soln).lower(),
                str(recent_rtcm).lower(),
                int(msg.satellites),
                float(msg.horizontal_accuracy_m),
                float(msg.vertical_accuracy_m),
            )
        )

    def publish_invalid(self) -> None:
        """Publish an invalid GNSS sample."""
        msg = GnssAbs()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.valid = False
        msg.corrected = False
        msg.fixed = False
        self.publisher.publish(msg)

    def has_recent_correction(self) -> bool:
        """Return true when correction bytes were received recently."""
        if self.last_correction_rx_time is None:
            return False

        return (time.monotonic() - self.last_correction_rx_time) <= self.correction_freshness_sec

    def describe_fix_type(self, fix_type: int) -> str:
        """Convert UBX fix type code to a readable label."""
        labels = {
            0: "none",
            1: "dead_reckoning_only",
            2: "2d",
            3: "3d",
            4: "gnss_dead_reckoning",
            5: "time_only",
        }
        return labels.get(fix_type, f"unknown({fix_type})")

    def describe_carrier_solution(self, carr_soln: int) -> str:
        """Convert UBX carrier solution code to a readable label."""
        labels = {
            0: "none",
            1: "float",
            2: "fixed",
        }
        return labels.get(carr_soln, f"unknown({carr_soln})")

    def build_poll_message(self, message_class: int, message_id: int) -> bytes:
        """Build a UBX poll frame for the requested message."""
        body = bytes([message_class, message_id, 0x00, 0x00])
        return b"\xB5\x62" + body + self.ubx_checksum(body)

    def ubx_checksum(self, data: bytes) -> bytes:
        """Compute a UBX two-byte checksum."""
        ck_a = 0
        ck_b = 0

        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        return bytes([ck_a, ck_b])

    def extract_ubx_packet(
        self,
        buffer: bytes,
        message_class: int,
        message_id: int,
    ) -> Optional[bytes]:
        """Return the first valid UBX packet matching the class and id."""
        signature = bytes([0xB5, 0x62, message_class, message_id])
        index = 0

        while True:
            start = buffer.find(signature, index)

            if start < 0 or start + 8 > len(buffer):
                return None

            payload_length = struct.unpack_from("<H", buffer, start + 4)[0]
            end = start + 6 + payload_length + 2

            if end > len(buffer):
                return None

            packet = buffer[start:end]
            body = packet[2:-2]

            if self.ubx_checksum(body) == packet[-2:]:
                return packet

            index = start + 1

    def parse_nav_pvt(self, buffer: bytes) -> Optional[dict]:
        """Parse a UBX-NAV-PVT message from a mixed serial buffer."""
        packet = self.extract_ubx_packet(buffer, 0x01, 0x07)

        if packet is None:
            return None

        payload = packet[6:-2]

        if len(payload) < self.NAV_PVT_PAYLOAD_LENGTH:
            return None

        flags = payload[21]

        return {
            "fix_type": int(payload[20]),
            "fix_ok": bool(flags & 0x01),
            "diff_soln": bool(flags & 0x02),
            "carr_soln": int((flags >> 6) & 0x03),
            "satellites": int(payload[23]),
            "longitude": struct.unpack_from("<i", payload, 24)[0] * 1e-7,
            "latitude": struct.unpack_from("<i", payload, 28)[0] * 1e-7,
            "altitude_m": struct.unpack_from("<i", payload, 36)[0] / 1000.0,
            "horizontal_accuracy_m": struct.unpack_from("<I", payload, 40)[0] / 1000.0,
            "vertical_accuracy_m": struct.unpack_from("<I", payload, 44)[0] / 1000.0,
        }

    def parse_nav_hpposllh(self, buffer: bytes) -> Optional[dict]:
        """Parse a UBX-NAV-HPPOSLLH message from a mixed serial buffer."""
        packet = self.extract_ubx_packet(buffer, 0x01, 0x14)

        if packet is None:
            return None

        payload = packet[6:-2]

        if len(payload) < self.NAV_HPPOSLLH_PAYLOAD_LENGTH:
            return None

        longitude = struct.unpack_from("<i", payload, 8)[0]
        latitude = struct.unpack_from("<i", payload, 12)[0]
        h_msl = struct.unpack_from("<i", payload, 20)[0]

        return {
            "invalid": bool(payload[3] & 0x01),
            "longitude": (longitude + struct.unpack_from("<b", payload, 24)[0] * 1e-2) * 1e-7,
            "latitude": (latitude + struct.unpack_from("<b", payload, 25)[0] * 1e-2) * 1e-7,
            "altitude_m": (h_msl + struct.unpack_from("<b", payload, 27)[0] * 0.1) / 1000.0,
            "horizontal_accuracy_m": struct.unpack_from("<I", payload, 28)[0] / 10000.0,
            "vertical_accuracy_m": struct.unpack_from("<I", payload, 32)[0] / 10000.0,
        }

    def _close_serial(self) -> None:
        """Close the serial port if it is open."""
        if self.serial_handle is None:
            return

        try:
            if self.serial_handle.is_open:
                self.serial_handle.close()
        except Exception:
            pass

        self.serial_handle = None

    def destroy_node(self):
        """Close device resources before node shutdown."""
        self._close_serial()
        return super().destroy_node()


def main(args=None):
    """Start the GNSS node."""
    rclpy.init(args=args)
    node = GnssNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
