#!/usr/bin/env python3

"""
Generic I2C service node for Raspberry Pi.

This node exposes a ROS 2 service that performs basic I2C operations through
smbus2. It optionally selects a PCA9548A-like I2C multiplexer channel before
accessing the target device.
"""

import os
import threading
from typing import List

import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from robot_interfaces.srv import I2COperation


class I2CSrvNode(Node):
    """
    Provides a generic ROS 2 service interface for I2C read/write operations.

    Operation behavior:
        - read_length == 0  -> write only
        - read_length > 0   -> read operation

    The 'operation' field is informational only and can contain values such as:
        relay, encoder, steer, compass, etc.
    """

    def __init__(self):
        """
        Initialize the I2C service node, validate the default I2C device path,
        and create the ROS 2 service.
        """
        super().__init__("i2c_srv_node", automatically_declare_parameters_from_overrides=True)

        self.default_bus_path = "/dev/i2c-1"
        self.bus_lock = threading.Lock()

        if not os.path.exists(self.default_bus_path):
            self.get_logger().error(
                f"I2C bus device not found: {self.default_bus_path}. "
                "Enable I2C on Raspberry Pi or check the bus number."
            )
        else:
            self.get_logger().info(f"I2C bus device found: {self.default_bus_path}")

        self.service = self.create_service(
            I2COperation,
            "/i2c/operation",
            self.i2c_operation_callback
        )

        self.get_logger().info("i2c_srv_node started. Service: /i2c/operation")

    def i2c_operation_callback(self, request: I2COperation.Request, response: I2COperation.Response):
        """
        Handle an I2C operation request.

        Logic:
            - read_length == 0 : write only
            - read_length > 0  : read only

        Args:
            request: Incoming ROS 2 service request.
            response: Outgoing ROS 2 service response.

        Returns:
            I2COperation.Response
        """
        try:
            self.validate_request(request)

            with self.bus_lock:
                with SMBus(request.bus_id) as bus:

                    if request.use_mux:
                        self.select_mux_channel(
                            bus=bus,
                            mux_address=request.mux_address,
                            mux_channel=request.mux_channel
                        )

                    if request.read_length == 0:
                        self.write_data(
                            bus=bus,
                            device_address=request.device_address,
                            register_address=request.register_address,
                            payload=list(request.payload)
                        )
                        response.data = []

                    else:
                        response.data = self.read_data(
                            bus=bus,
                            device_address=request.device_address,
                            register_address=request.register_address,
                            read_length=request.read_length
                        )

            response.success = True
            response.error = ""

        except Exception as ex:
            response.success = False
            response.error = str(ex)
            response.data = []

            device_name = request.device_name if request.device_name else "unknown_device"

            self.get_logger().error(
                f"I2C operation failed for {device_name}: {ex}"
            )

        return response

    def validate_request(self, request: I2COperation.Request) -> None:
        """
        Validate request parameters.

        Args:
            request: Incoming request.

        Raises:
            ValueError
            FileNotFoundError
        """
        bus_path = f"/dev/i2c-{request.bus_id}"

        if not os.path.exists(bus_path):
            raise FileNotFoundError(f"I2C bus device not found: {bus_path}")

        if request.use_mux and request.mux_channel > 7:
            raise ValueError("mux_channel must be between 0 and 7")

        if request.read_length == 0 and len(request.payload) == 0:
            raise ValueError(
                "Write mode selected (read_length=0), but payload is empty"
            )

    def select_mux_channel(self, bus: SMBus, mux_address: int, mux_channel: int) -> None:
        """
        Select active channel on PCA9548A-like multiplexer.

        Args:
            bus: SMBus instance.
            mux_address: MUX I2C address.
            mux_channel: Channel index (0..7).
        """
        channel_mask = 1 << mux_channel
        bus.write_byte(mux_address, channel_mask)

    def read_data(
        self,
        bus: SMBus,
        device_address: int,
        register_address: int,
        read_length: int
    ) -> List[int]:
        """
        Read bytes from target register.

        Args:
            bus: SMBus instance.
            device_address: Target device address.
            register_address: Register address.
            read_length: Number of bytes.

        Returns:
            List[int]
        """
        if read_length == 1:
            value = bus.read_byte_data(device_address, register_address)
            return [value]

        return bus.read_i2c_block_data(
            device_address,
            register_address,
            read_length
        )

    def write_data(
        self,
        bus: SMBus,
        device_address: int,
        register_address: int,
        payload: List[int]
    ) -> None:
        """
        Write bytes to target register.

        Args:
            bus: SMBus instance.
            device_address: Target device address.
            register_address: Register address.
            payload: Bytes to write.
        """
        if len(payload) == 1:
            bus.write_byte_data(
                device_address,
                register_address,
                payload[0]
            )
            return

        bus.write_i2c_block_data(
            device_address,
            register_address,
            payload
        )


def main(args=None):
    """
    Start ROS 2 node.
    """
    rclpy.init(args=args)

    node = I2CSrvNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()