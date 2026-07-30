#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from robot_interfaces.msg import Gnss

from robot_core.Tools import _get_param
from robot_core.gnss_controller import GnssController


class GnssRoverNode(Node):
    def __init__(self):
        super().__init__(
            "gnss_rover_node",
            automatically_declare_parameters_from_overrides=True,
        )

        self.serial_port = str(
            _get_param(
                self,
                "serial_port",
                (
                    "/dev/serial/by-id/"
                    "usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
                ),
            )
        )
        self.baudrate = int(_get_param(self, "baudrate", 460800))
        self.serial_timeout_s = float(
            _get_param(self, "serial_timeout_s", 0.2)
        )
        self.measurement_rate_hz = float(
            _get_param(self, "measurement_rate_hz", 10.0)
        )
        self.publish_period_s = float(
            _get_param(self, "publish_period_s", 0.1)
        )
        self.log_period_s = float(_get_param(self, "log_period_s", 5.0))
        self._last_log_time = 0.0
        self.rtcm_log_period_s = float(_get_param(self, "rtcm_log_period_s", 2.0))
        self._last_rtcm_log_time = 0.0
        self._rtcm_frame_count = 0

        self.rtcm_topic = str(_get_param(self, "rtcm_topic", "/station/rtcm"))
        self.rtcm_sub = self.create_subscription(
            UInt8MultiArray,
            self.rtcm_topic,
            self.rtcm_callback,
            10,
        )

        self.gnss_pub = self.create_publisher(Gnss, "/robot/gnss", 10)

        self.gnss = GnssController(
            self.serial_port,
            self.baudrate,
            serial_timeout_s=self.serial_timeout_s,
            measurement_rate_hz=self.measurement_rate_hz,
        )

        is_gnss_operational = False
        try:
            if self.gnss.connect():
                self.get_logger().info("gnss rover device connected.")
                if self.gnss.configure_rover():
                    self.get_logger().info("gnss rover device configured.")
                    is_gnss_operational = True
                else:
                    self.get_logger().warn(
                        "gnss rover device can not configured!!!"
                    )
            else:
                self.get_logger().warn(
                    "gnss rover device can not connected!!!"
                )
        except Exception as exc:
            self.get_logger().warn(f"gnss rover node startup failed: {exc}")

        if is_gnss_operational:
            self.get_logger().info("gnss_rover_node up and running...")
        else:
            self.get_logger().warn("gnss_rover_node down!!!")

        self.create_timer(self.publish_period_s, self.publish_gnss_callback)

    def rtcm_callback(self, msg: UInt8MultiArray) -> None:
        payload = bytes(msg.data)
        self.gnss.send_rtcm(payload)
        self._rtcm_frame_count += 1

        now = time.monotonic()
        if now - self._last_rtcm_log_time >= self.rtcm_log_period_s:
            self._last_rtcm_log_time = now


    def publish_gnss_callback(self) -> None:
        pos = self.gnss.get_position()

        msg = Gnss()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = float(pos["latitude"])
        msg.longitude = float(pos["longitude"])
        msg.altitude = float(pos["altitude"])
        msg.accuracy_m = float(pos["accuracy_m"])
        msg.vertical_accuracy_m = float(pos["vertical_accuracy_m"])
        msg.satellite_count = int(pos["satellite_count"])
        msg.fix_type = str(pos["fix_type"])
        msg.ground_speed_m_s = float(pos["ground_speed_m_s"])
        msg.heading_deg = float(pos["heading_deg"])
        msg.speed_accuracy_m_s = float(pos["speed_accuracy_m_s"])
        msg.heading_accuracy_deg = float(pos["heading_accuracy_deg"])
        msg.pdop = float(pos["pdop"])
        msg.fix_ok = bool(pos["fix_ok"])
        msg.differential_solution = bool(pos["differential_solution"])
        self.gnss_pub.publish(msg)

        now = time.monotonic()
        if now - self._last_log_time >= self.log_period_s:
            
            self._last_log_time = now
            self.get_logger().info(
                f"Mod: {msg.fix_type} | Uydu: {msg.satellite_count} "
                f"| fix_type: {msg.fix_type}"
                f"| Hassasiyet: {msg.accuracy_m} m "
                f"| Hiz: {msg.ground_speed_m_s} m/s "
                f"| Yon: {msg.heading_deg} deg | Enlem: {msg.latitude} "
                f"| Boylam: {msg.longitude}"
            )

    def destroy_node(self):
        try:
            self.gnss.disconnect()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = GnssRoverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
