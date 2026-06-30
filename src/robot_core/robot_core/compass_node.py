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
from rclpy.parameter import Parameter


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
        
        self.bus_id = int(self._get_param("bus_id", 1))
        self.device_name = str(self._get_param("device_name", "compass"))
        self.device_address = self._get_param("device_address", 0x1E)

        self.use_mux = bool(self._get_param("use_mux", False))
        self.mux_address = 0
        self.mux_channel = 0

        self.frequency = self._get_param("frequency", 10.0)

        self.publisher = self.create_publisher(Compass, "/robot/compass", 10)
        self.pending_read_future = None

        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i2c_client = self.create_client(I2COperation, "/i2c/operation")

        self._wait_for_service(self.i2c_client)

        self.configure_sensor()

        self.get_logger().info("compass_node started")

    def _get_param(self, name: str, default_value=None):
        """
        Generic ROS2 parameter reader.

        - If parameter exists -> returns plain Python value
        - If not exists -> declares with default and returns value
        - If a Parameter object is encountered -> unwraps it
        - Works for int, float, bool, str, list, etc.
        """

        try:
            if self.has_parameter(name):
                param = self.get_parameter(name)
            else:
                param = self.declare_parameter(name, default_value)
        except Exception:
            # fallback path
            param = self.get_parameter(name)

        # If already plain python type
        if not isinstance(param, Parameter):
            return param

        # Standard ROS2 Parameter object
        try:
            return param.value
        except Exception:
            pass

        # Deep fallback for Humble edge cases
        try:
            pv = param.get_parameter_value()

            if pv.integer_value != 0 or isinstance(default_value, int):
                return pv.integer_value
            if pv.double_value != 0.0 or isinstance(default_value, float):
                return pv.double_value
            if pv.bool_value in (True, False):
                return pv.bool_value
            if pv.string_value != "":
                return pv.string_value
            if pv.integer_array_value:
                return list(pv.integer_array_value)
            if pv.double_array_value:
                return list(pv.double_array_value)
            if pv.bool_array_value:
                return list(pv.bool_array_value)
            if pv.string_array_value:
                return list(pv.string_array_value)

        except Exception:
            pass

        return default_value

            
    
    def _wait_for_service(self, client) -> None:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for I2C manager service..."
            )

    def configure_sensor(self) -> None:
        """
        Configure HMC5883L style sensor.
        """
        self.write_register(self.REG_CONFIG_A, 0x70)
        self.write_register(self.REG_CONFIG_B, 0x20)
        self.write_register(self.REG_MODE, 0x00)

        self.get_logger().info("Compass configured")

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
        request.bus_id = self.bus_id
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
        request.bus_id = self.bus_id
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
    """
    Start compass node.
    """
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
