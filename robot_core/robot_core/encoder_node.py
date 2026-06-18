#!/usr/bin/env python3

"""
Steering encoder node.

This node reads four AS5600 magnetic encoders through the generic I2C service
and publishes their raw 12-bit steering angle values at the configured rate.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from robot_interfaces.srv import I2COperation
from rclpy.parameter import Parameter


class EncoderNode(Node):
    """
    Reads steering encoder values through the generic I2C service.

    The node sends read requests to /i2c/operation and publishes 
    the resulting encoder values on /robot/steering_encoders.
    """

    def __init__(self):
        """
        Initialize parameters, service client, publisher, and timer.
        """
        super().__init__("encoder_node", automatically_declare_parameters_from_overrides=True)

        self.bus_id = int(self._get_param("bus_id", 1))
        self.device_name = str(self._get_param("device_name", "AS5600"))
        self.device_address = int(self._get_param("device_address", 0x36))

        self.use_mux = bool(self._get_param("use_mux", True))
        self.mux_address = int(self._get_param("mux_address", 0x70))

        self.frequency = float(self._get_param("frequency", 10.0))

        self.motor_channels = [
            int(self._get_param("motor1_sd", 7)),
            int(self._get_param("motor2_sd", 6)),
            int(self._get_param("motor3_sd", 4)),
            int(self._get_param("motor4_sd", 5)),
        ]

        self.angle_high_register = 0x0C

        self.i2c_client = self.create_client(I2COperation, "/i2c/operation")
        self.pending_read_future = None
        self.pending_channels = []
        self.pending_values = []
        self.current_mux_channel = None

        self._wait_for_service(self.i2c_client)

        self.publisher = self.create_publisher(Int32MultiArray, "/robot/steering_encoders", 10)

        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("encoder_node started")

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


    def read_raw_angle_async(self, mux_channel: int):
        """
        Send an asynchronous two-byte raw angle read through the generic I2C service.

        Args:
            mux_channel: PCA9548A channel number.
        """
        request = I2COperation.Request()

        request.device_name = self.device_name
        request.bus_id = self.bus_id

        request.use_mux = self.use_mux
        request.mux_address = self.mux_address
        request.mux_channel = mux_channel

        request.device_address = self.device_address
        request.operation = "encoder"

        request.register_address = self.angle_high_register
        request.read_length = 2
        request.payload = []

        return self.i2c_client.call_async(request)

    def extract_raw_angle(self, future) -> int:
        """
        Validate a completed I2C read future and return the raw 12-bit angle.
        """
        response = future.result()

        if response is None:
            raise RuntimeError("I2C service returned no response")

        if not response.success:
            raise RuntimeError(response.error)

        if len(response.data) < 2:
            raise RuntimeError("I2C service returned fewer than 2 bytes")

        return int(((int(response.data[0]) << 8) | int(response.data[1])) & 0x0FFF)

    def start_next_encoder_read(self) -> None:
        """
        Start reading the next encoder channel in the current scan cycle.
        """
        if not self.pending_channels:
            msg = Int32MultiArray()
            msg.data = self.pending_values
            self.publisher.publish(msg)

            self.pending_values = []
            self.current_mux_channel = None
            return

        self.current_mux_channel = self.pending_channels.pop(0)

        self.pending_read_future = self.read_raw_angle_async(self.current_mux_channel)
        self.pending_read_future.add_done_callback(self.handle_raw_angle_response)


    def handle_raw_angle_response(self, future) -> None:
        """
        Finish one encoder value and continue with the next channel.
        """
        self.pending_read_future = None

        try:
            raw_angle = self.extract_raw_angle(future)
            self.pending_values.append(raw_angle)
            self.start_next_encoder_read()



        except Exception as ex:
            self.handle_read_error(ex)

    def handle_read_error(self, ex: Exception) -> None:
        """
        Reset the current scan state after an asynchronous read failure.
        """
        self.pending_read_future = None
        self.pending_channels = []
        self.pending_values = []
        self.current_mux_channel = None

        self.get_logger().error(f"Encoder read error: {ex}")

    def timer_callback(self) -> None:
        """
        Start one asynchronous encoder scan if no request is in flight.
        """
        if not self.i2c_client.service_is_ready():
            self.get_logger().warn("Waiting for /i2c/operation service...")
            return

        if self.pending_read_future is not None or self.current_mux_channel is not None:
            return

        self.pending_channels = list(self.motor_channels)
        self.pending_values = []
        self.start_next_encoder_read()


def main(args=None):
    """
    Start the steering encoder node.
    """
    rclpy.init(args=args)

    node = EncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
