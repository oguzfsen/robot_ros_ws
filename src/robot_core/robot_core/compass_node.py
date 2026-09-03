#!/usr/bin/env python3

"""
Compass node.

Reads a root-bus I2C magnetometer and publishes heading data at fixed rate.
Designed for common HMC5883L / QMC5883 compatible modules.
"""

import math

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Compass
from robot_interfaces.srv import I2COperation
from robot_core.Tools import RobotState, _get_param, _wait_for_service


class CompassNode(Node):
    """
    Reads magnetometer values through the generic I2C service and publishes
    heading information.
    """

    REG_CONFIG_A = 0x00
    REG_CONFIG_B = 0x01
    REG_MODE = 0x02

    REG_DATA_START = 0x03

    def __init__(self):
        """
        Initialize node, parameters, I2C client, sensor config, publisher, timer.
        """
        super().__init__("compass_node", automatically_declare_parameters_from_overrides=True)
        
        self.device_name = str(_get_param(self, "device_name", "compass"))
        self.device_address = int(_get_param(self, "device_address", 0x1E))
        self.use_mux = bool(_get_param(self, "use_mux", False))
        self.topic_compass = str(_get_param(self, "topic_compass", "/robot/compass"))
        self.control_frequency = float(_get_param(self, "control_frequency", 10.0))
        self.mux_address = 0
        self.mux_channel = 0

        self.publisher = self.create_publisher(Compass, self.topic_compass, 10)
        self.pending_read_future = None
        
        self.timer = self.create_timer(1.0 / self.control_frequency, self.timer_callback)

        self.i2c_client = self.create_client(I2COperation, "/i2c/operation")
        _wait_for_service(self, self.i2c_client)

        self.configure_sensor()



    def configure_sensor(self) -> None:
        """
        Configure HMC5883L style sensor.
        """
        self.write_register(self.REG_CONFIG_A, 0x70)
        self.write_register(self.REG_CONFIG_B, 0x20)
        self.write_register(self.REG_MODE, 0x00)


    def timer_callback(self) -> None:
        """
        Start one asynchronous compass read if no request is in flight.
        """
        if self.pending_read_future is not None:
            return

        self.pending_read_future = self.read_block_async(
            self.REG_DATA_START,
            6
        )
        self.pending_read_future.add_done_callback(self.handle_read_block_response)

    def to_int16(self, high: int, low: int) -> int:
        """
        Convert two bytes to signed int16.
        """
        value = (high << 8) | low

        if value >= 32768:
            value -= 65536

        return value

    def read_block_async(self, register_address: int, read_length: int):
        """
        Send an asynchronous block-read request through the generic I2C service.
        """
        request = I2COperation.Request()

        request.device_name = self.device_name
        request.bus_id = 1
        request.use_mux = self.use_mux
        request.mux_address = self.mux_address
        request.mux_channel = self.mux_channel
        request.device_address = self.device_address
        request.operation = "compass"

        request.register_address = register_address
        request.read_length = read_length
        request.payload = []

        return self.i2c_client.call_async(request)

    def handle_read_block_response(self, future) -> None:
        """
        Handle a completed asynchronous compass read.
        """
        self.pending_read_future = None

        try:
            response = future.result()

            if response is None:
                raise RuntimeError("No response from I2C service")

            if not response.success:
                raise RuntimeError(response.error)

            data = response.data

            if len(data) < 6:
                raise RuntimeError("Compass read returned incomplete data")

            x = self.to_int16(data[0], data[1])
            z = self.to_int16(data[2], data[3])
            y = self.to_int16(data[4], data[5])

            heading_rad = math.atan2(y, x)
            heading_deg = math.degrees(heading_rad)

            if heading_deg < 0:
                heading_deg += 360.0

            msg = Compass()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.heading_deg = float(heading_deg)
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            msg.valid = True

            self.publisher.publish(msg)

        except Exception as ex:
            self.get_logger().error(f"Compass read error: {ex}")

            msg = Compass()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.valid = False

            self.publisher.publish(msg)

    def write_register(self, register_address: int, value: int) -> None:
        """
        Write one byte through generic I2C service.
        """
        request = I2COperation.Request()

        request.device_name = self.device_name
        request.bus_id = 1
        request.use_mux = self.use_mux
        request.mux_address = self.mux_address
        request.mux_channel = self.mux_channel
        request.device_address = self.device_address
        request.operation = "compass"

        request.register_address = register_address
        request.read_length = 0
        request.payload = [value & 0xFF]

        future = self.i2c_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        response = future.result()

        if response is None:
            raise RuntimeError("No response from I2C service")

        if not response.success:
            raise RuntimeError(response.error)


def main(args=None):
    rclpy.init(args=args)

    node = CompassNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
