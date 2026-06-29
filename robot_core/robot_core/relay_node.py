#!/usr/bin/env python3

"""
Relay controller node.

This node listens to /robot/relay and controls relay outputs through an
SX1509 GPIO expander by using the generic /i2c/operation service.
"""

from typing import List

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import RobotRelay
from robot_interfaces.srv import I2COperation
from Tools import _get_param


class RelayNode(Node):
    """
    Controls ignition, reverse, brake, rear power, and alarm relays
    through an SX1509 GPIO expander.

    The node only writes to the I2C bus when the requested relay state changes.
    It does not access the I2C bus directly; all I2C transactions are sent to
    the generic /i2c/operation service.
    """

    REG_DIR_B = 0x0E
    REG_DIR_A = 0x0F
    REG_DATA_B = 0x10
    REG_DATA_A = 0x11

    def __init__(self):
        """
        Initialize parameters, I2C service client, relay state, and subscriber.
        """
        super().__init__("relay_node", automatically_declare_parameters_from_overrides=True)

        self.bus_id = int(_get_param(self, "bus_id", 1))
        self.device_name = str(_get_param(self, "device_name", "sx1509_gpio"))
        self.device_address = int(_get_param(self, "device_address", 0x3E))

        self.use_mux = bool(_get_param(self, "use_mux", False))
        self.mux_address = int(_get_param(self, "mux_address", 0))
        self.mux_channel = int(_get_param(self, "mux_channel", 0))

        self.active_low = bool(_get_param(self, "active_low", True))

        self.ignition_pin = int(_get_param(self, "ignition_pin", 0))
        self.topic_name = str(_get_param(self, "topic_name", "/robot/relay"))

        self.reverse_pins = [
            int(_get_param(self, "reverse_pin1", 1)),
            int(_get_param(self, "reverse_pin2", 2)),
            int(_get_param(self, "reverse_pin3", 3)),
            int(_get_param(self, "reverse_pin4", 13)),
        ]

        self.brake_pin = int(_get_param(self, "brake_pin", 12))
        self.back_energy_pin = int(_get_param(self, "back_energy", 14))
        self.alarm_lamp_pin = int(_get_param(self, "alarm_lamp", 15))

        self.output_pins = [
            self.ignition_pin,
            *self.reverse_pins,
            self.brake_pin,
            self.back_energy_pin,
            self.alarm_lamp_pin,
        ]

        self.last_ignition = None
        self.last_reverse = None
        self.last_brake = None
        self.last_back_energy = None
        self.last_alarm_lamp = None
        self.desired_ignition = None
        self.desired_reverse = None
        self.desired_brake = None
        self.desired_back_energy = None
        self.desired_alarm_lamp = None

        self.data_a = 0xFF
        self.data_b = 0xFF
        self.dir_a = 0xFF
        self.dir_b = 0xFF

        self.i2c_client = self.create_client(I2COperation, "/i2c/operation")
        self.pending_write_future = None
        self.active_write_state = None
        self.active_write_data_a = None
        self.active_write_data_b = None

        self._wait_for_service(self.i2c_client)

        self.configure_outputs()
        self.write_all_outputs_safe()

        self.subscription = self.create_subscription(RobotRelay, self.topic_name, self.relay_callback, 10)

        self.get_logger().info("relay_controller_node started")


    def _wait_for_service(self, client) -> None:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for I2C manager service..."
            )

    def relay_callback(self, msg: RobotRelay) -> None:
        """
        Handle incoming relay command messages.

        Args:
            msg: RobotRelay message containing relay states.
        """
        changed = (
            self.last_ignition != msg.ignition or
            self.last_reverse != msg.reverse or
            self.last_brake != msg.brake or
            self.last_back_energy != msg.back_energy or
            self.last_alarm_lamp != msg.alarm_lamp
        )

        if not changed:
            return

        try:
            self.desired_ignition = msg.ignition
            self.desired_reverse = msg.reverse
            self.desired_brake = msg.brake
            self.desired_back_energy = msg.back_energy
            self.desired_alarm_lamp = msg.alarm_lamp

            self.set_pin_group([self.ignition_pin], msg.ignition)
            self.set_pin_group(self.reverse_pins, msg.reverse)
            self.set_pin_group([self.brake_pin], msg.brake)
            self.set_pin_group([self.back_energy_pin], msg.back_energy)
            self.set_pin_group([self.alarm_lamp_pin], msg.alarm_lamp)

            if self.pending_write_future is None and self.active_write_state is None:
                self.start_write_data_registers_async()

        except Exception as ex:
            self.get_logger().error(f"Relay update failed: {ex}")

    def configure_outputs(self) -> None:
        """
        Configure all relay pins as SX1509 outputs.

        In SX1509 direction registers:
            0 = output
            1 = input
        """
        for pin in self.output_pins:
            self.validate_pin(pin)

            if pin < 8:
                self.dir_a &= ~(1 << pin)
            else:
                self.dir_b &= ~(1 << (pin - 8))

        self.write_register_sync(self.REG_DIR_A, self.dir_a)
        self.write_register_sync(self.REG_DIR_B, self.dir_b)

    def write_all_outputs_safe(self) -> None:
        """
        Initialize all relay outputs to the inactive state.
        """
        inactive_level = True if self.active_low else False

        for pin in self.output_pins:
            self.set_physical_pin_level(pin, inactive_level)

        self.write_register_sync(self.REG_DATA_A, self.data_a)
        self.write_register_sync(self.REG_DATA_B, self.data_b)

    def set_pin_group(self, pins: List[int], active: bool) -> None:
        """
        Set a group of logical relay outputs.

        Args:
            pins: SX1509 pin list.
            active: Logical relay state.
        """
        for pin in pins:
            physical_level = self.logical_to_physical(active)
            self.set_physical_pin_level(pin, physical_level)

    def logical_to_physical(self, active: bool) -> bool:
        """
        Convert logical relay state to physical GPIO level.

        Args:
            active: Logical relay state.

        Returns:
            bool: Physical output level.
        """
        if self.active_low:
            return not active

        return active

    def set_physical_pin_level(self, pin: int, high: bool) -> None:
        """
        Set one SX1509 pin level in the local register cache.

        Args:
            pin: SX1509 pin number, 0 to 15.
            high: Physical output level.
        """
        self.validate_pin(pin)

        if pin < 8:
            if high:
                self.data_a |= (1 << pin)
            else:
                self.data_a &= ~(1 << pin)
        else:
            bit = pin - 8

            if high:
                self.data_b |= (1 << bit)
            else:
                self.data_b &= ~(1 << bit)

    def start_write_data_registers_async(self) -> None:
        """
        Start an asynchronous write of the cached SX1509 data registers.
        """
        self.active_write_state = (
            self.desired_ignition,
            self.desired_reverse,
            self.desired_brake,
            self.desired_back_energy,
            self.desired_alarm_lamp,
        )
        self.active_write_data_a = self.data_a
        self.active_write_data_b = self.data_b

        self.pending_write_future = self.write_register_async(
            self.REG_DATA_A,
            self.active_write_data_a
        )
        self.pending_write_future.add_done_callback(self.handle_data_a_write_response)

    def build_write_request(self, register_address: int, value: int) -> I2COperation.Request:
        """
        Build an I2C write request for the SX1509.
        """
        request = I2COperation.Request()

        request.device_name = self.device_name
        request.bus_id = self.bus_id

        request.use_mux = self.use_mux
        request.mux_address = self.mux_address
        request.mux_channel = self.mux_channel

        request.device_address = self.device_address
        request.operation = "relay"

        request.register_address = register_address
        request.read_length = 0
        request.payload = [value & 0xFF]

        return request

    def write_register_async(self, register_address: int, value: int):
        """
        Send an asynchronous one-byte SX1509 register write.
        """
        request = self.build_write_request(register_address, value)
        return self.i2c_client.call_async(request)

    def write_register_sync(self, register_address: int, value: int) -> None:
        """
        Write one byte to an SX1509 register synchronously.

        Args:
            register_address: SX1509 register address.
            value: One-byte value to write.

        Raises:
            RuntimeError: If the I2C service fails.
        """
        request = self.build_write_request(register_address, value)
        future = self.i2c_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if not future.done():
            raise RuntimeError("I2C service timeout")

        self.validate_write_response(future)

    def validate_write_response(self, future) -> None:
        """
        Validate a completed SX1509 write future.
        """
        response = future.result()

        if response is None:
            raise RuntimeError("I2C service returned no response")

        if not response.success:
            raise RuntimeError(response.error)

    def handle_data_a_write_response(self, future) -> None:
        """
        Validate DATA_A write, then start DATA_B write.
        """
        self.pending_write_future = None

        try:
            self.validate_write_response(future)

            self.pending_write_future = self.write_register_async(
                self.REG_DATA_B,
                self.active_write_data_b
            )
            self.pending_write_future.add_done_callback(self.handle_data_b_write_response)

        except Exception as ex:
            self.handle_write_error(ex)

    def handle_data_b_write_response(self, future) -> None:
        """
        Finalize a relay update after DATA_B write completes.
        """
        self.pending_write_future = None

        try:
            self.validate_write_response(future)

            (
                self.last_ignition,
                self.last_reverse,
                self.last_brake,
                self.last_back_energy,
                self.last_alarm_lamp,
            ) = self.active_write_state

            self.get_logger().info(
                f"Relay updated: ignition={self.last_ignition}, "
                f"reverse={self.last_reverse}, brake={self.last_brake}, "
                f"back_energy={self.last_back_energy}, alarm_lamp={self.last_alarm_lamp}"
            )

            self.active_write_state = None
            self.active_write_data_a = None
            self.active_write_data_b = None

            if (
                self.desired_ignition != self.last_ignition or
                self.desired_reverse != self.last_reverse or
                self.desired_brake != self.last_brake or
                self.desired_back_energy != self.last_back_energy or
                self.desired_alarm_lamp != self.last_alarm_lamp
            ):
                self.start_write_data_registers_async()

        except Exception as ex:
            self.handle_write_error(ex)

    def handle_write_error(self, ex: Exception) -> None:
        """
        Reset the active async write state after a relay write failure.
        """
        self.pending_write_future = None
        self.active_write_state = None
        self.active_write_data_a = None
        self.active_write_data_b = None

        self.get_logger().error(f"Relay update failed: {ex}")

    def validate_pin(self, pin: int) -> None:
        """
        Validate SX1509 pin number.

        Args:
            pin: SX1509 pin number.

        Raises:
            ValueError: If pin is outside 0..15.
        """
        if pin < 0 or pin > 15:
            raise ValueError(f"SX1509 pin must be between 0 and 15, got {pin}")


def main(args=None):
    rclpy.init(args=args)

    node = RelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
