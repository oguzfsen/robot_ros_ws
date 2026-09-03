#!/usr/bin/env python3

"""
Steering zero calibration node.

This node is an independent calibration utility. It listens to RC input,
manually drives one selected steering motor through PCA9685, reads the related
AS5600 raw encoder value directly over I2C, and saves the zero value to the
database once per second.
"""

import time
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from robot_interfaces.msg import RcInput
from robot_interfaces.srv import EncoderZeroSet
from std_msgs.msg import Int32MultiArray
from robot_core.Tools import _get_param, _wait_for_service


class SteerZeroNode(Node):
    """
    Provides RC-based manual steering zero calibration.

    This node does not use the generic I2C service. It directly accesses:
        - PCA9685 PWM driver on root I2C bus
        - PCA9548A mux for AS5600 encoders
        - AS5600 magnetic encoders
    """

    PCA9685_ADDRESS = 0x40
    PCA9685_MODE1 = 0x00
    PCA9685_PRESCALE = 0xFE
    PCA9685_LED0_ON_L = 0x06

    MUX_ADDRESS = 0x70
    AS5600_ADDRESS = 0x36
    AS5600_RAW_HIGH = 0x0C
    AS5600_RAW_LOW = 0x0D

    PWM_FREQUENCY = 50

    
    SAVE_INTERVAL_SEC = 1.0

    def __init__(self):
        super().__init__("steer_zero_node", automatically_declare_parameters_from_overrides=True)

        self.topic_enc_zero_set = str(_get_param(self, "topic_enc_zero_set", "/database/encoder_zero_set"))
        self.steer_input_deadband = int(_get_param(self, "steer_input_deadband", 5))
        self.steering_slow_pulse = int(_get_param(self, "steering_slow_pulse", 2000))


        self.steer_front_left_reverse = int(_get_param(self, "steer_front_left_reverse", 5))
        self.steer_front_left_forward = int(_get_param(self, "steer_front_left_forward", 4))
        self.steer_front_right_reverse = int(_get_param(self, "steer_front_right_reverse", 1))
        self.steer_front_right_forward = int(_get_param(self, "steer_front_right_forward", 0))
        self.steer_rear_left_reverse = int(_get_param(self, "steer_rear_left_reverse", 7))
        self.steer_rear_left_forward = int(_get_param(self, "steer_rear_left_forward", 6))
        self.steer_rear_right_reverse = int(_get_param(self, "steer_rear_right_reverse", 3))
        self.steer_rear_right_forward = int(_get_param(self, "steer_rear_right_forward", 2))

        self.forward_pins = [
            self.steer_front_left_forward,
            self.steer_front_right_forward,
            self.steer_rear_left_forward,
            self.steer_rear_right_forward]
        
        self.reverse_pins = [
            self.steer_front_left_reverse,
            self.steer_front_right_reverse,
            self.steer_rear_left_reverse,
            self.steer_rear_right_reverse]


        self.encoder_pins = [
            int(_get_param(self, "encoder1", 4)),
            int(_get_param(self, "encoder2", 5)),
            int(_get_param(self, "encoder3", 6)),
            int(_get_param(self, "encoder4", 7)),
        ]

        self.steering_center = 0

        self.bus = SMBus(1)
        self.last_rc: RcInput | None = None
        self.selected_motor_index: int | None = None
        self.last_saved_time = 0.0

        self.encoder_zero_client = self.create_client(EncoderZeroSet, self.topic_enc_zero_set)
        _wait_for_service(self, self.encoder_zero_client)

        self.configure_pca9685()
        self.stop_all_motors()

        self.rc_subscription = self.create_subscription(RcInput, "/rc/input", self.rc_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("steer_zero_node started")
        self.get_logger().info(
            "Selection logic: crab<40=left, crab>60=right, "
            "reverse<2=front, reverse==2=rear"
        )
    

    def rc_callback(self, msg: RcInput) -> None:
        """
        Store latest RC input message.

        Args:
            msg: RC input message.
        """
        self.last_rc = msg

    def timer_callback(self) -> None:
        """
        Main calibration loop.

        It selects one steering motor, drives it according to RC steering input,
        reads encoder raw value, and periodically saves it to the database.
        """
        if self.last_rc is None:
            return

        if not self.last_rc.connected:
            self.stop_all_motors()
            self.selected_motor_index = None
            return

        motor_index = self.select_motor_index(
            crab=self.last_rc.crab,
            reverse=self.last_rc.reverse
        )

        if motor_index is None:
            self.stop_all_motors()
            self.selected_motor_index = None
            return

        self.selected_motor_index = motor_index

        try:
            self.drive_selected_motor(
                motor_index=motor_index,
                steering=self.last_rc.steering
            )

            raw_value = self.read_encoder_raw(motor_index)
        except Exception as ex:
            self.stop_all_motors()
            self.get_logger().error(f"Steering zero I2C error: {ex}")
            return

        now = time.time()

        if now - self.last_saved_time >= self.SAVE_INTERVAL_SEC:
            self.save_encoder_zero(
                motor_index=motor_index,
                raw_value=raw_value
            )
            self.last_saved_time = now

    def select_motor_index(self, crab: int, reverse: int) -> int | None:
        """
        Select steering motor index from RC switch values.

        Args:
            crab: RC crab channel value.
            reverse: RC reverse channel value.

        Returns:
            Optional[int]: Motor index 0..3, or None if selection is invalid.

        Mapping:
            0 = front left
            1 = front right
            2 = rear left
            3 = rear right
        """
        if crab < 40:
            side = "left"
        elif crab > 60:
            side = "right"
        else:
            return None

        if reverse < 2:
            axle = "front"
        elif reverse == 2:
            axle = "rear"
        else:
            return None

        if axle == "front" and side == "left":
            return 0

        if axle == "front" and side == "right":
            return 1

        if axle == "rear" and side == "left":
            return 2

        if axle == "rear" and side == "right":
            return 3

        return None

    def drive_selected_motor(self, motor_index: int, steering: int) -> None:
        """
        Drive only the selected steering motor from RC steering input.

        Args:
            motor_index: Selected motor index.
            steering: RC steering value, expected around 0..100.
        """
        error = steering - self.steering_center

        for index in range(4):
            if index != motor_index:
                self.stop_motor(index)

        if abs(error) <= self.steer_input_deadband:
            self.stop_motor(motor_index)
            return

        if error > 0:
            self.drive_motor_forward(motor_index, self.steering_slow_pulse)
        else:
            self.drive_motor_reverse(motor_index, self.steering_slow_pulse)

    def read_encoder_raw(self, motor_index: int) -> int:
        """
        Read AS5600 raw value for selected steering motor.

        Args:
            motor_index: Motor index 0..3.

        Returns:
            int: 12-bit AS5600 raw value.
        """
        enc_pin = self.encoder_pins[motor_index]

        self.select_mux_channel(enc_pin)

        data = self.bus.read_i2c_block_data(
            self.AS5600_ADDRESS,
            self.AS5600_RAW_HIGH,
            2
        )
        high = data[0]
        low = data[1]

        return int(((high << 8) | low) & 0x0FFF)

    def save_encoder_zero(self, motor_index: int, raw_value: int) -> None:
        """
        Save selected encoder zero raw value to the database.

        Args:
            motor_index: Motor index 0..3.
            raw_value: Raw AS5600 value.
        """
        request = EncoderZeroSet.Request()
        request.motor_id = motor_index + 1
        request.raw_value = int(raw_value)

        future = self.encoder_zero_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.encoder_zero_save_callback(
                completed_future,
                motor_index,
                raw_value
            )
        )

    def encoder_zero_save_callback(self, future, motor_index: int, raw_value: int) -> None:
        """
        Handle database save result.

        Args:
            future: Completed service future.
            motor_index: Motor index 0..3.
            raw_value: Saved raw value.
        """
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f"Saved zero: motor{motor_index + 1}, raw={raw_value}"
                )
            else:
                self.get_logger().error(
                    f"Database save failed: {response.message}"
                )

        except Exception as ex:
            self.get_logger().error(f"Database save service error: {ex}")

    def select_mux_channel(self, mux_channel: int) -> None:
        """
        Select PCA9548A mux channel.

        Args:
            mux_channel: Channel number 0..7.
        """
        self.bus.write_byte(self.MUX_ADDRESS, 1 << mux_channel)

    def configure_pca9685(self) -> None:
        """
        Configure PCA9685 PWM frequency.
        """
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.PCA9685_MODE1, 0x00)

        prescale = int(round(25000000.0 / (4096.0 * self.PWM_FREQUENCY) - 1.0))

        old_mode = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.PCA9685_MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10

        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.PCA9685_MODE1, sleep_mode)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.PCA9685_PRESCALE, prescale)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.PCA9685_MODE1, old_mode | 0xA1)

        self.get_logger().info(
            f"PCA9685 configured directly: frequency={self.PWM_FREQUENCY} Hz, prescale={prescale}"
        )

    def set_pwm(self, channel: int, pulse: int) -> None:
        """
        Set PCA9685 PWM output.

        Args:
            channel: PCA9685 channel number.
            pulse: OFF count from 0 to 4095.
        """
        pulse = max(0, min(4095, int(pulse)))

        register_address = self.PCA9685_LED0_ON_L + 4 * channel

        payload = [
            0x00,
            0x00,
            pulse & 0xFF,
            (pulse >> 8) & 0x0F,
        ]

        self.bus.write_i2c_block_data(
            self.PCA9685_ADDRESS,
            register_address,
            payload
        )

    def drive_motor_forward(self, motor_index: int, pulse: int) -> None:
        """
        Drive selected steering motor forward.

        Args:
            motor_index: Motor index 0..3.
            pulse: PWM pulse.
        """
        self.set_pwm(self.reverse_pins[motor_index], 0)
        self.set_pwm(self.forward_pins[motor_index], pulse)

    def drive_motor_reverse(self, motor_index: int, pulse: int) -> None:
        """
        Drive selected steering motor reverse.

        Args:
            motor_index: Motor index 0..3.
            pulse: PWM pulse.
        """
        self.set_pwm(self.forward_pins[motor_index], 0)
        self.set_pwm(self.reverse_pins[motor_index], pulse)

    def stop_motor(self, motor_index: int) -> None:
        """
        Stop selected steering motor.

        Args:
            motor_index: Motor index 0..3.
        """
        self.set_pwm(self.forward_pins[motor_index], 0)
        self.set_pwm(self.reverse_pins[motor_index], 0)

    def stop_all_motors(self) -> None:
        """
        Stop all steering motors.
        """
        for motor_index in range(4):
            self.stop_motor(motor_index)

    def close(self) -> None:
        """
        Stop motors and close I2C bus.
        """
        self.stop_all_motors()
        self.bus.close()


def main(args=None):
    """
    Start steering zero calibration node.
    """
    rclpy.init(args=args)

    node = SteerZeroNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
