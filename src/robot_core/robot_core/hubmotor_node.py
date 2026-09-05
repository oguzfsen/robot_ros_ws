#!/usr/bin/env python3

"""
Hub motor node.

This node listens to hub motor power commands and writes four MCP4725 DAC
outputs through the generic /i2c/operation service. The subscriber callback
only stores the target state. A timer loop sends DAC writes one by one so that
only one I2C request is in flight at a time.
"""

import time

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import HubMotor, Gnss, Malfunction
from robot_interfaces.srv import I2COperation
from robot_core.Tools import _get_param, _wait_for_service, clamp_int


class HubMotorNode(Node):
    """Control four hub motor DAC outputs through MCP4725 devices."""

    MCP4725_WRITE_DAC = 0x40
    MCP4725_WRITE_DAC_AND_EEPROM = 0x60

    def __init__(self):
        """Initialize parameters, I2C client, address detection, and timers."""
        super().__init__("hubmotor_node", automatically_declare_parameters_from_overrides=True)

        self.topic_hub_motor = str(_get_param(self, "topic_hub_motor", "/robot/hub_motor"))
        self.topic_gnss = str(_get_param(self, "topic_gnss", "/robot/gnss"))
        self.device_name = str(_get_param(self, "device_name", "mcp4725"))
        self.topic_malfunction = str(_get_param(self, "topic_malfunction", "/robot/malfunction"))

        self.use_mux = bool(_get_param(self, "use_mux", True))
        self.mux_address = int(_get_param(self, "mux_address", 113))
        self.primary_address = int( _get_param(self, "primary_address", 96))
        self.secondary_address = int(_get_param(self, "secondary_address", 97 ))
        self.min_dac_value = int(_get_param(self, "min_dac_value", 0))
        self.max_dac_value = int(_get_param(self, "max_dac_value", 4095))
        self.max_power_percentage = int(_get_param(self, "max_power_percentage", 100))
        self.command_timeout_sec = float(_get_param(self, "command_timeout_sec", 0.5))
        self.watchdog_frequency = float(_get_param(self, "watchdog_frequency", 10.0))
        self.speed_limit = float(_get_param(self, "speed_limit", 1.5))
        self.i2c_timeout_sec = float(_get_param(self, "encoder_timeout_sec", 0.5))
        self.i2c_malfunction_timeout = float(
            _get_param(self, "encoder_malfunction_timeout", 5.0)
        )
        self.min_dac_value = clamp_int(self.min_dac_value, 0, 4095)
        self.max_dac_value = clamp_int(self.max_dac_value, self.min_dac_value, 4095)
        self.max_power_percentage = clamp_int(self.max_power_percentage, 0, 100)
        self.command_timeout_sec = max(0.1, self.command_timeout_sec)


        self.motor_order = [
            "front_left",
            "front_right",
            "rear_left",
            "rear_right",
        ]

        self.motors = {
            "front_left": {
                "mux_channel": int(_get_param(self, "front_left_mux_channel", 2)),
                "device_address": None,
            },
            "front_right": {
                "mux_channel": int(_get_param(self, "front_right_mux_channel", 3)),
                "device_address": None,
            },
            "rear_left": {
                "mux_channel": int(_get_param(self, "rear_left_mux_channel", 4)),
                "device_address": None,
            },
            "rear_right": {
                "mux_channel": int(_get_param(self, "rear_right_mux_channel", 5)),
                "device_address": None,
            },
        }

        self.enabled = False
        self.desired_power_percentages = {motor_name: 0 for motor_name in self.motor_order}
        self.desired_dac_values = {motor_name: 0 for motor_name in self.motor_order}
        self.applied_dac_values = {motor_name: 0 for motor_name in self.motor_order}

        self.pending_i2c_future = None
        self.active_motor_name = None
        self.active_dac_value = None
        self.last_command_time = time.monotonic()
        self.watchdog_logged = False
        self.malfunction_active = False

        self.i2c_error_start_sec = {
            motor_name: None for motor_name in self.motor_order
        }
        self.i2c_timeout_logged = {
            motor_name: False for motor_name in self.motor_order
        }

        self.malfunction_publisher = self.create_publisher(
            Malfunction,
            self.topic_malfunction,
            10,
        )

        self.i2c_client = self.create_client(I2COperation, "/i2c/operation")
        _wait_for_service(self, self.i2c_client)

        try:
            self.detect_device_addresses()
            self.initialize_eeprom_to_zero()
        except Exception as ex:
            self.trigger_malfunction(f"Hub motor initialization failed: {ex}")

        self.subscription = self.create_subscription(HubMotor, self.topic_hub_motor, self.hubmotor_callback, 10)
        self.watchdog_timer = self.create_timer(1.0 / self.watchdog_frequency, self.watchdog_callback)

        self.gnss: Gnss | None = None
        self.gnss_subscription = self.create_subscription(Gnss, self.topic_gnss, self.gnss_callback, 10)


    def gnss_callback(self, msg: Gnss) -> None:
        self.gnss = msg

#region Power Management
    def hubmotor_callback(self, msg: HubMotor) -> None:
        """Store the latest hub motor targets from the incoming message."""
        self.last_command_time = time.monotonic()
        self.watchdog_logged = False

        if self.malfunction_active:
            self.stop_all_motors()
            self.start_next_i2c_write()
            return

        if self.is_too_fast():
            self.set_motor_power("front_left", 0)
            self.set_motor_power("front_right", 0)
            self.set_motor_power("rear_left", 0)
            self.set_motor_power("rear_right", 0)
        else:
            self.set_motor_power("front_left", int(msg.front_left))
            self.set_motor_power("front_right", int(msg.front_right))
            self.set_motor_power("rear_left", int(msg.rear_left))
            self.set_motor_power("rear_right", int(msg.rear_right))

        self.start_next_i2c_write()

    def is_too_fast(self) -> bool:
        if self.gnss is None:
            return False

        return (
            self.gnss.speed_accuracy_m_s < 0.3
            and self.gnss.ground_speed_m_s > self.speed_limit
        )

    def control_callback(self) -> None:
        """Process pending DAC writes in the timer loop."""

        if self.malfunction_active:
            self.stop_all_motors()
            self.start_next_i2c_write()
            return

        self.start_next_i2c_write()

    def watchdog_callback(self) -> None:
        """Force hub motors to zero if commands stop arriving."""

        if (time.monotonic() - self.last_command_time) <= self.command_timeout_sec:
            return

        self.stop_all_motors()
        self.start_next_i2c_write()

        if not self.watchdog_logged:
            self.get_logger().warning(
                "Hub motor command watchdog expired, forcing all motors to zero"
            )
            self.watchdog_logged = True

    def set_motor_power(self, motor_name: str, raw_power: int) -> None:
        """Convert one motor power command into a safe DAC target."""
        power_percentage = self.normalize_power_percentage(raw_power)
        dac_value = self.power_percentage_to_dac_value(power_percentage)

        self.desired_power_percentages[motor_name] = power_percentage
        self.desired_dac_values[motor_name] = dac_value

    def stop_all_motors(self) -> None:
        """Set all desired motor outputs to zero."""
        for motor_name in self.motor_order:
            self.desired_power_percentages[motor_name] = 0
            self.desired_dac_values[motor_name] = 0

    def stop_all_motors_sync(self) -> None:
        """Write zero DAC output to all motors immediately."""
        for motor_name in self.motor_order:
            self.write_dac_value_sync(motor_name, 0)
            self.desired_power_percentages[motor_name] = 0
            self.desired_dac_values[motor_name] = 0
            self.applied_dac_values[motor_name] = 0

    def normalize_power_percentage(self, raw_power: int) -> int:
        """Convert signed or unsigned input into a safe 0..100 power value.

        The DAC output is treated as a throttle magnitude. Zero means stop.
        """
        limited_power = clamp_int(raw_power, -100, 100)
        return abs(limited_power)

    def power_percentage_to_dac_value(self, power_percentage: int) -> int:
        """Map a 0..100 power percentage to a DAC value."""
        power_percentage = clamp_int(power_percentage, 0, 100)

        if power_percentage == 0:
            return 0

        allowed_power = (power_percentage * self.max_power_percentage) / 100.0
        dac_range = self.max_dac_value - self.min_dac_value
        dac_value = self.min_dac_value + int(round(dac_range * allowed_power / 100.0))

        return clamp_int(dac_value, 0, 4095)

    def trigger_malfunction(self, error: str) -> None:
        if self.malfunction_active:
            return

        self.malfunction_active = True
        self.stop_all_motors()
        self.start_next_i2c_write()

        msg = Malfunction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.malfunction = True
        msg.error = error
        self.malfunction_publisher.publish(msg)
        self.get_logger().error(error)

    def reset_i2c_error_check(self, motor_name: str) -> None:
        self.i2c_error_start_sec[motor_name] = None
        self.i2c_timeout_logged[motor_name] = False

    def handle_i2c_write_error(self, motor_name: str, error: str) -> None:
        now_sec = time.monotonic()
        error_start_sec = self.i2c_error_start_sec[motor_name]

        if error_start_sec is None:
            self.i2c_error_start_sec[motor_name] = now_sec
            return

        error_duration_sec = now_sec - error_start_sec

        if error_duration_sec >= self.i2c_malfunction_timeout:
            self.trigger_malfunction(
                "Hub motor I2C not responding: "
                f"motor={motor_name} "
                f"timeout_s={self.i2c_malfunction_timeout:.1f} "
                f"error={error}"
            )
            return

        if (
            error_duration_sec >= self.i2c_timeout_sec
            and not self.i2c_timeout_logged[motor_name]
        ):
            self.get_logger().warning(
                "Hub motor I2C write is temporarily unavailable: "
                f"motor={motor_name} error={error}"
            )
            self.i2c_timeout_logged[motor_name] = True

#endregion

#region I2C Communication
    def detect_device_addresses(self) -> None:
        """Detect whether each MCP4725 is on the primary or secondary address."""
        for motor_name in self.motor_order:
            detected_address = self.detect_motor_address(motor_name)
            self.motors[motor_name]["device_address"] = detected_address


    def detect_motor_address(self, motor_name: str) -> int:
        """Try both possible device addresses and return the first working one."""
        last_error = "unknown error"

        for device_address in [self.primary_address, self.secondary_address]:
            self.motors[motor_name]["device_address"] = device_address

            try:
                self.write_dac_value_sync(motor_name, 0)
                return device_address
            except Exception as ex:
                last_error = str(ex)

        raise RuntimeError(
            f"Could not detect MCP4725 address for {motor_name}: {last_error}"
        )

    def initialize_eeprom_to_zero(self) -> None:
        """Store zero in each MCP4725 EEPROM for safe power-on behavior."""
        for motor_name in self.motor_order:
            self.write_eeprom_value_sync(motor_name, 0)
            time.sleep(0.05)

            self.desired_power_percentages[motor_name] = 0
            self.desired_dac_values[motor_name] = 0
            self.applied_dac_values[motor_name] = 0

    def build_i2c_write_request(
        self,
        motor_name: str,
        register_address: int,
        payload: list[int]
    ) -> I2COperation.Request:
        """Build one I2C write request for the selected motor."""
        request = I2COperation.Request()
        request.device_name = self.device_name
        request.bus_id = 1
        request.use_mux = self.use_mux
        request.mux_address = self.mux_address if self.use_mux else 0
        request.mux_channel = self.motors[motor_name]["mux_channel"] if self.use_mux else 0
        request.device_address = int(self.motors[motor_name]["device_address"])
        request.operation = "hubmotor"
        request.register_address = register_address
        request.read_length = 0
        request.payload = payload
        return request

    def build_mcp4725_payload(self, dac_value: int) -> list[int]:
        """Split a 12-bit DAC value into two MCP4725 data bytes."""
        dac_value = clamp_int(dac_value, 0, 4095)
        return [
            (dac_value >> 4) & 0xFF,
            (dac_value & 0x0F) << 4,
        ]

    def validate_i2c_response(self, future) -> None:
        """Raise an exception when the I2C service response is not successful."""
        response = future.result()

        if response is None:
            raise RuntimeError("I2C service returned no response")

        if not response.success:
            raise RuntimeError(response.error)

    def i2c_write_sync(self, motor_name: str, register_address: int, payload: list[int]) -> bool:
        """Send one synchronous I2C write request."""
        request = self.build_i2c_write_request(motor_name, register_address, payload)
        future = self.i2c_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done():
            raise TimeoutError("I2C request timeout")

        self.validate_i2c_response(future)
        return True

    def i2c_write_async(self, motor_name: str, register_address: int, payload: list[int]):
        """Send one asynchronous I2C write request."""
        request = self.build_i2c_write_request(motor_name, register_address, payload)
        return self.i2c_client.call_async(request)

    def write_dac_value_sync(self, motor_name: str, dac_value: int) -> bool:
        """Write a DAC value without touching EEPROM."""
        payload = self.build_mcp4725_payload(dac_value)
        return self.i2c_write_sync(motor_name, self.MCP4725_WRITE_DAC, payload)

    def write_eeprom_value_sync(self, motor_name: str, dac_value: int) -> bool:
        """Write a DAC value and save it into EEPROM."""
        payload = self.build_mcp4725_payload(dac_value)
        return self.i2c_write_sync(motor_name, self.MCP4725_WRITE_DAC_AND_EEPROM, payload)

    def write_dac_value_async(self, motor_name: str, dac_value: int):
        """Start an asynchronous DAC write for one motor."""
        payload = self.build_mcp4725_payload(dac_value)
        return self.i2c_write_async(motor_name, self.MCP4725_WRITE_DAC, payload)

    def start_next_i2c_write(self) -> None:
        """Start the next pending DAC write if no other request is active."""
        if self.pending_i2c_future is not None or self.active_motor_name is not None:
            return

        if not self.i2c_client.service_is_ready():
            return

        for motor_name in self.motor_order:
            desired_dac_value = self.desired_dac_values[motor_name]
            applied_dac_value = self.applied_dac_values[motor_name]

            if desired_dac_value == applied_dac_value:
                self.reset_i2c_error_check(motor_name)
                continue

            self.active_motor_name = motor_name
            self.active_dac_value = desired_dac_value
            self.pending_i2c_future = self.write_dac_value_async(motor_name, desired_dac_value)
            self.pending_i2c_future.add_done_callback(self.handle_i2c_write_response)
            return

    def handle_i2c_write_response(self, future) -> None:
        """Finish one asynchronous write and continue with the next one."""
        motor_name = self.active_motor_name
        dac_value = self.active_dac_value

        self.pending_i2c_future = None
        self.active_motor_name = None
        self.active_dac_value = None

        try:
            self.validate_i2c_response(future)

            if motor_name is not None and dac_value is not None:
                self.applied_dac_values[motor_name] = dac_value
                self.reset_i2c_error_check(motor_name)

            self.start_next_i2c_write()

        except Exception as ex:
            if motor_name is not None:
                self.handle_i2c_write_error(motor_name, str(ex))

#endregion


def main(args=None) -> None:
    """Start the hub motor node."""
    rclpy.init(args=args)

    node = HubMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_all_motors_sync()
        except Exception as ex:
            node.get_logger().warning(f"Failed to stop hub motors on shutdown: {ex}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
