#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from robot_interfaces.msg import Steer, Malfunction
from robot_interfaces.srv import I2COperation, EncoderZeroGet
from robot_core.Tools import _get_param
from typing import List
import time

class SteerNode(Node):

    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE

    LED0_ON_L = 0x06    

    def __init__(self):
        super().__init__("steer_node", automatically_declare_parameters_from_overrides=True)

        self.device_name = str(_get_param(self, "device_name", "pca9685_pwm"))
        self.bus_id = int(_get_param(self, "bus_id", 1))
        self.device_address = int(_get_param(self, "device_address", 0x40))
        self.pwm_frequency = float(_get_param(self, "frequency", 50.0))
        self.control_frequency = float(_get_param(self, "control_frequency", 10.0))
        self.deadband_deg = float(_get_param(self, "deadband_deg", 2.0))
        self.encoder_timeout_sec = float(_get_param(self, "encoder_timeout_sec", 0.5))
        self.encoder_zero_response_timeout_sec = float(
            _get_param(self, "encoder_zero_response_timeout_sec", 2.0)
        )
        self.encoder_zero_startup_wait_sec = float(
            _get_param(self, "encoder_zero_startup_wait_sec", 20.0)
        )
        self.max_steer_angle_deg = float(_get_param(self, "max_steer_angle_deg", 60.0))
        self.drive_pulse_1 = int(_get_param(self, "drive_pulse_1", 3000))
        self.drive_pulse_2 = int(_get_param(self, "drive_pulse_2", 3000))
        self.drive_pulse_3 = int(_get_param(self, "drive_pulse_3", 3000))
        self.drive_pulse_4 = int(_get_param(self, "drive_pulse_4", 3000))
        self.fake_pid = int(_get_param(self, "fake_pid", 10.0))
        self.no_response_duration_s = float(_get_param(self, "no_response_duration_s", 3.0))
        # Treat tiny encoder jitter as no movement for stall detection.
        self.no_response_min_delta_deg = max(1.0, self.deadband_deg)


        self.front_left_forward_pin = int(_get_param(self, "front_left_forward_pin", 5))
        self.front_right_forward_pin = int(_get_param(self, "front_right_forward_pin", 1))
        self.rear_left_forward_pin = int(_get_param(self, "rear_left_forward_pin", 7))
        self.rear_right_forward_pin = int(_get_param(self, "rear_right_forward_pin", 3))
        self.front_left_reverse_pin = int(_get_param(self, "front_left_reverse_pin", 4))
        self.front_right_reverse_pin = int(_get_param(self, "front_right_reverse_pin", 0))
        self.rear_left_reverse_pin = int(_get_param(self, "rear_left_reverse_pin", 6))
        self.rear_right_reverse_pin = int(_get_param(self, "rear_right_reverse_pin", 2))
        self.database_listen_client = str(_get_param(self, "database_listen_client", "/database/encoder_zero_get"))
        self.encoder_listen_topic = str(_get_param(self, "encoder_listen_topic", "/robot/steering_encoders"))
        self.core_listen_topic = str(_get_param(self, "core_listen_topic", "/robot/steer"))
        self.malfunction_topic = str(_get_param(self, "malfunction_topic", "/robot/malfunction"))

        self.drive_pulses = [
            self.drive_pulse_1,
            self.drive_pulse_2,
            self.drive_pulse_3,
            self.drive_pulse_4
        ]

        self.motor_state = {
            1: "",
            2: "",
            3: "",
            4: "",
        }

        self.motor_pwm = {
            1: 0,
            2: 0,
            3: 0,
            4: 0,
        }

        self.encoder_zero_client = self.create_client(EncoderZeroGet, self.database_listen_client)
        self._wait_for_service(self.encoder_zero_client)
        self.encoder_zero_values = self.read_db_with_retry()
        self.get_logger().info(str(self.encoder_zero_values))
        self.enabled = False
        self.target_angles = [0, 0, 0, 0]
        self.encoder_ready = False
        self.last_encoder_update_sec = 0.0
        self.waiting_for_encoder_logged = False
        self.encoder_timeout_logged = False
        self.malfunction_active = False
        now_sec = time.monotonic()
        self.motor_response_state = {
            motor_id: {
                "command": "stop",
                "last_angle_deg": None,
                "last_motion_sec": now_sec,
            }
            for motor_id in range(1, 5)
        }

        self.encoder_values = []
        self.enc_subs = self.create_subscription(Int32MultiArray, self.encoder_listen_topic, self.encoder_callback, 10)
        self.steer_subs = self.create_subscription(Steer, self.core_listen_topic, self.steer_callback, 10)
        self.mulfunction_publisher = self.create_publisher(Malfunction, self.malfunction_topic, 10)


        self.motor_pins = {
            1: {
                "reverse": self.front_left_reverse_pin,
                "forward": self.front_left_forward_pin,
            },
            2: {
                "reverse": self.front_right_reverse_pin,
                "forward": self.front_right_forward_pin,
            },
            3: {
                "reverse": self.rear_left_reverse_pin,
                "forward": self.rear_left_forward_pin,
            },
            4: {
                "reverse": self.rear_right_reverse_pin,
                "forward": self.rear_right_forward_pin,
            },
        }
        self.pwm_channels = [
            self.front_left_forward_pin,
            self.front_left_reverse_pin,
            self.front_right_forward_pin,
            self.front_right_reverse_pin,
            self.rear_left_forward_pin,
            self.rear_left_reverse_pin,
            self.rear_right_forward_pin,
            self.rear_right_reverse_pin,
        ]

        self.desired_channel_payloads = {
            channel: self.build_pwm_off_payload()
            for channel in self.pwm_channels
        }
        self.applied_channel_payloads = {
            channel: self.build_pwm_off_payload()
            for channel in self.pwm_channels
        }
        self.pending_i2c_future = None
        self.active_i2c_channel = None
        self.active_i2c_payload = None

        self.i2c_client = self.create_client(I2COperation, "/i2c/operation")
        self._wait_for_service(self.i2c_client)
        
        self.init_pca9685(self.pwm_frequency)
        for motor_id in range(1, 5):
            self.motor_state[motor_id] = "stop"
            self.motor_pwm[motor_id] = 0

        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.control_callback)


        self.enc_pub_timer = self.create_timer(1.0, self.enc_pub_callback)
        self.get_logger().info(
            f"SteerNode started. control_frequency={self.control_frequency} Hz, "
            f"pwm_frequency={self.pwm_frequency} Hz"
        )

    def enc_pub_callback(self) -> None:
        enc_deggree = [self.to_degree(x) for x in self.encoder_values]
        enc_zero = [self.to_degree(x) for x in self.encoder_zero_values]

        enc_fark = [a - b for a, b in zip(enc_deggree, enc_zero)]
        
        enc_normal = [self.normalize_angle(x) for x in enc_fark]
        # self.get_logger().info(str(enc_normal))
    
    def encoder_callback(self, encoder_values:Int32MultiArray) -> None:
        if len(encoder_values.data) < 4:
            self.get_logger().warn("Encoder message has fewer than 4 values")
            return

        self.encoder_values = list(encoder_values.data[:4])
        self.encoder_ready = True
        self.last_encoder_update_sec = time.monotonic()
        self.waiting_for_encoder_logged = False
        self.encoder_timeout_logged = False


#region Steering
    def steer_callback(self, msg:Steer) -> None:
        if self.malfunction_active:
            self.enabled = False
            self.stop_all_motors()
            self.start_next_i2c_write()
            return

        self.enabled = bool(msg.enable)
        
        self.target_angles = [
            msg.front_left_angle,
            msg.front_right_angle,
            msg.rear_left_angle,
            msg.rear_right_angle
        ]
        
        # (-100,+100) aralığından (-75,+75) aralığına map'lanıyor.
        self.target_angles = [self.map_range(x, -100, 100, -self.max_steer_angle_deg, self.max_steer_angle_deg) for x in self.target_angles]

        if not self.enabled:
            self.stop_all_motors()
            self.start_next_i2c_write()


    def control_callback(self) -> None:
        self.start_next_i2c_write()

        if self.malfunction_active:
            self.stop_all_motors()
            self.start_next_i2c_write()
            return

        if not self.enabled:
            self.reset_all_motor_response_checks()
            self.stop_all_motors()
            self.start_next_i2c_write()
            return

        if not self.encoder_ready:
            if not self.waiting_for_encoder_logged:
                self.get_logger().warn("Waiting for steering encoder data...")
                self.waiting_for_encoder_logged = True
            self.reset_all_motor_response_checks()
            return

        if (time.monotonic() - self.last_encoder_update_sec) > self.encoder_timeout_sec:
            if not self.encoder_timeout_logged:
                self.get_logger().warn("Steering encoder data timed out")
                self.encoder_timeout_logged = True
            self.encoder_ready = False
            self.reset_all_motor_response_checks()
            self.stop_all_motors()
            self.start_next_i2c_write()
            return

        try:
            for motor_id, target_deg in enumerate(self.target_angles, start=1):
                self.steer_to_angle(motor_id, target_deg)
                if self.malfunction_active:
                    self.start_next_i2c_write()
                    return
            self.start_next_i2c_write()
        except Exception as ex:
            self.get_logger().error(f"Steering control error: {ex}")
            self.reset_all_motor_response_checks()
            self.stop_all_motors()
            self.start_next_i2c_write()


    def steer_to_angle(self, motor_id: int, target_deg: int) -> None:
        if not (-90 <= target_deg <= 90):
            self.get_logger().warn(
                f"Motor {motor_id}: target {target_deg}° out of safe range [-90, 90], skipping"
            )
            return

        raw_enc = self.encoder_values[motor_id - 1]
        raw_zero = self.encoder_zero_values[motor_id - 1]

        enc_angle = self.to_degree(raw_enc)
        zero_angle = self.to_degree(raw_zero)

        current_deg = self.normalize_angle(enc_angle - zero_angle)
        error_deg = self.normalize_angle(target_deg - current_deg)

        if error_deg < -self.deadband_deg:
            command = "left"
        elif error_deg > self.deadband_deg:
            command = "right"
        else:
            command = "stop"

        if self.check_motor_no_response(
            motor_id,
            current_deg,
            target_deg,
            error_deg,
            command,
        ):
            return

        if command == "left":
            self.turn_left(motor_id, error_deg)
        elif command == "right":
            self.turn_right(motor_id, error_deg)
        else:
            self.stop_motor(motor_id)


    def to_degree(self, raw_value: int) -> int:
        return int(round(raw_value * 360.0 / 4095.0))
    

    def normalize_angle(self, angle_deg: int) -> int:
        return (angle_deg + 180) % 360 - 180
    
    def turn_left(self, motor_id: int, error_deg: int) -> None:
        drive_pulse = self.drive_pulses[motor_id - 1]
        if abs(error_deg) < self.fake_pid:
            drive_pulse = int(drive_pulse/2.0)

        if (
            self.motor_state[motor_id] == "left" and
            self.motor_pwm[motor_id] == drive_pulse
        ):
            return

        pins = self.motor_pins[motor_id]

        self.set_pwm_off(pins["forward"])
        self.set_pwm(pins["reverse"], drive_pulse)

        self.motor_state[motor_id] = "left"
        self.motor_pwm[motor_id] = drive_pulse


    def turn_right(self, motor_id: int, error_deg: int) -> None:
        drive_pulse = self.drive_pulses[motor_id - 1]
        if abs(error_deg) < self.fake_pid:
            drive_pulse = int(drive_pulse/2.0)

        if (
            self.motor_state[motor_id] == "right" and
            self.motor_pwm[motor_id] == drive_pulse
        ):
            return

        pins = self.motor_pins[motor_id]

        self.set_pwm_off(pins["reverse"])
        self.set_pwm(pins["forward"], drive_pulse)

        self.motor_state[motor_id] = "right"
        self.motor_pwm[motor_id] = drive_pulse


    def stop_all_motors(self) -> None:
        for i in range(1,5):
            self.stop_motor(i)
        

    def stop_motor(self, motor_id: int) -> None:
        if self.motor_state[motor_id] == "stop":
            return

        pins = self.motor_pins[motor_id]

        self.set_pwm_off(pins["forward"])
        self.set_pwm_off(pins["reverse"])

        self.motor_state[motor_id] = "stop"
        self.motor_pwm[motor_id] = 0

    def check_motor_no_response(
        self,
        motor_id: int,
        current_deg: int,
        target_deg: int,
        error_deg: int,
        command: str,
    ) -> bool:
        state = self.motor_response_state[motor_id]
        now_sec = time.monotonic()

        if command == "stop":
            self.reset_motor_response_check(motor_id, current_deg=current_deg, now_sec=now_sec)
            return False

        if state["last_angle_deg"] is None:
            self.reset_motor_response_check(
                motor_id,
                current_deg=current_deg,
                now_sec=now_sec,
                command=command,
            )
            return False

        angle_delta = abs(self.normalize_angle(current_deg - state["last_angle_deg"]))
        if angle_delta >= self.no_response_min_delta_deg:
            state["last_angle_deg"] = current_deg
            state["last_motion_sec"] = now_sec

        if state["command"] != command:
            self.reset_motor_response_check(
                motor_id,
                current_deg=current_deg,
                now_sec=now_sec,
                command=command,
            )
            return False

        if (now_sec - state["last_motion_sec"]) < self.no_response_duration_s:
            return False

        self.trigger_malfunction(
            "Steering encoder not responding: "
            f"motor={motor_id} command={command} "
            f"target_deg={target_deg} current_deg={current_deg} "
            f"error_deg={error_deg} timeout_s={self.no_response_duration_s:.1f}"
        )
        return True

    def reset_motor_response_check(
        self,
        motor_id: int,
        current_deg: int | None = None,
        now_sec: float | None = None,
        command: str = "stop",
    ) -> None:
        if now_sec is None:
            now_sec = time.monotonic()

        self.motor_response_state[motor_id]["command"] = command
        self.motor_response_state[motor_id]["last_angle_deg"] = current_deg
        self.motor_response_state[motor_id]["last_motion_sec"] = now_sec

    def reset_all_motor_response_checks(self) -> None:
        now_sec = time.monotonic()
        for motor_id in range(1, 5):
            self.reset_motor_response_check(motor_id, now_sec=now_sec)

    def trigger_malfunction(self, error: str) -> None:
        if self.malfunction_active:
            return

        self.malfunction_active = True
        self.enabled = False
        self.reset_all_motor_response_checks()
        self.stop_all_motors()
        self.start_next_i2c_write()

        msg = Malfunction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.malfunction = True
        msg.error = error
        self.mulfunction_publisher.publish(msg)
        self.get_logger().error(error)

#endregion

#region I2C Communication

    def channel_register(self, channel: int) -> int:
        return self.LED0_ON_L + 4 * channel

    def init_pca9685(self, pwm_freq: float = 500.0) -> None:
        prescale = round(25_000_000 / (4096 * pwm_freq)) - 1
        prescale = max(3, min(255, prescale))

        self.get_logger().info(f"PCA9685 init, freq={pwm_freq} Hz, prescale={prescale}")

        # MODE1: sleep mode
        self.i2c_write_sync(self.MODE1, [0x10])
        time.sleep(0.005)

        # PWM frekansı
        self.i2c_write_sync(self.PRESCALE, [prescale])

        # MODE2: totem pole output / OUTDRV
        self.i2c_write_sync(self.MODE2, [0x04])

        # MODE1: auto-increment aktif
        self.i2c_write_sync(self.MODE1, [0x20])
        time.sleep(0.005)

        # RESTART + AUTO_INCREMENT
        self.i2c_write_sync(self.MODE1, [0xA0])
        time.sleep(0.005)

        self.stop_all_motors_sync()

        self.get_logger().info("PCA9685 configured successfully.")
    
    def build_i2c_write_request(self, register_address: int, payload: list[int]) -> I2COperation.Request:
        req = I2COperation.Request()
        req.device_name = self.device_name
        req.bus_id = self.bus_id
        req.use_mux = False
        req.mux_address = 0
        req.mux_channel = 0
        req.device_address = self.device_address
        req.operation = "steer"
        req.register_address = register_address
        req.read_length = 0
        req.payload = payload
        return req

    def validate_i2c_response(self, future) -> None:
        response = future.result()

        if response is None:
            raise RuntimeError("I2C service returned no response")

        if not response.success:
            raise RuntimeError(response.error)

    def i2c_write_sync(self, register_address: int, payload: list[int]) -> bool:
        req = self.build_i2c_write_request(register_address, payload)

        future = self.i2c_client.call_async(req)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done():
            raise TimeoutError("i2c request time error")

        self.validate_i2c_response(future)
        
        return True

    def i2c_write_async(self, register_address: int, payload: list[int]):
        req = self.build_i2c_write_request(register_address, payload)
        return self.i2c_client.call_async(req)

    def start_next_i2c_write(self) -> None:
        if self.pending_i2c_future is not None or self.active_i2c_channel is not None:
            return

        if not self.i2c_client.service_is_ready():
            return

        for channel in self.pwm_channels:
            desired_payload = self.desired_channel_payloads[channel]
            applied_payload = self.applied_channel_payloads[channel]

            if desired_payload == applied_payload:
                continue

            self.active_i2c_channel = channel
            self.active_i2c_payload = list(desired_payload)

            self.pending_i2c_future = self.i2c_write_async(
                self.channel_register(channel),
                self.active_i2c_payload
            )
            self.pending_i2c_future.add_done_callback(self.handle_i2c_write_response)
            return

    def handle_i2c_write_response(self, future) -> None:
        channel = self.active_i2c_channel
        payload = self.active_i2c_payload

        self.pending_i2c_future = None
        self.active_i2c_channel = None
        self.active_i2c_payload = None

        try:
            self.validate_i2c_response(future)

            if channel is not None and payload is not None:
                self.applied_channel_payloads[channel] = list(payload)

            self.start_next_i2c_write()

        except Exception as ex:
            self.get_logger().error(f"Steering I2C write failed: {ex}")

    def set_pwm(self, channel: int, pulse: int) -> None:
        pulse = max(0, min(4095, pulse))
        self.desired_channel_payloads[channel] = self.build_pwm_payload(pulse)
        self.start_next_i2c_write()
    
    def set_pwm_off(self, channel: int) -> None:
        self.desired_channel_payloads[channel] = self.build_pwm_off_payload()
        self.start_next_i2c_write()

    def set_pwm_sync(self, channel: int, pulse: int) -> None:
        pulse = max(0, min(4095, pulse))
        self.i2c_write_sync(self.channel_register(channel), self.build_pwm_payload(pulse))

    def set_pwm_off_sync(self, channel: int) -> None:
        self.i2c_write_sync(self.channel_register(channel), self.build_pwm_off_payload())

    def build_pwm_payload(self, pulse: int) -> list[int]:
        return [
            0x00,
            0x00,
            pulse & 0xFF,
            (pulse >> 8) & 0x0F
        ]

    def build_pwm_off_payload(self) -> list[int]:
        return [
            0x00,
            0x00,
            0x00,
            0x10
        ]

    def stop_all_motors_sync(self) -> None:
        for motor_id in range(1, 5):
            pins = self.motor_pins[motor_id]
            self.set_pwm_off_sync(pins["forward"])
            self.set_pwm_off_sync(pins["reverse"])

#endregion

#region Database
    def read_db(self) -> List[int]:
        """
        Load encoder zero raw values from the database service.

        Raises:
            RuntimeError: If the zero values cannot be read.
        """
        request = EncoderZeroGet.Request()
        future = self.encoder_zero_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.encoder_zero_response_timeout_sec,
        )

        if not future.done():
            raise RuntimeError("Encoder zero service timeout")

        response = future.result()
        if response is None:
            raise RuntimeError("Encoder zero service returned no response")
        if not response.success:
            raise RuntimeError(f"Encoder zero service failed: {response.message}")
        if len(response.zero_raw) < 4:
            raise RuntimeError("Encoder zero service returned fewer than 4 values")

        zero_values = [
            int(response.zero_raw[0]),
            int(response.zero_raw[1]),
            int(response.zero_raw[2]),
            int(response.zero_raw[3]),
        ]
        
        self.get_logger().info(
            "Encoder zero raw loaded: "
            f"m1={zero_values[0]}, "
            f"m2={zero_values[1]}, "
            f"m3={zero_values[2]}, "
            f"m4={zero_values[3]}"
        )
        
        return zero_values

    def read_db_with_retry(self) -> List[int]:
        deadline = time.monotonic() + max(1.0, self.encoder_zero_startup_wait_sec)
        last_error: Exception | None = None
        attempt = 0

        while rclpy.ok() and time.monotonic() < deadline:
            attempt += 1
            try:
                return self.read_db()
            except Exception as exc:
                last_error = exc
                self.get_logger().warn(
                    "Encoder zero service henuz cevap vermedi, tekrar denenecek: "
                    f"attempt={attempt} error={exc}"
                )
                time.sleep(0.5)

        raise RuntimeError(
            "Encoder zero values could not be loaded during startup"
            + (f": {last_error}" if last_error is not None else "")
        )
#endregion

#region Helper Functions
    
    def _wait_for_service(self, client) -> None:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for service: {client.srv_name}"
            )
    
    def map_range(self, value: int,
                in_min: int, in_max: int,
                out_min: int, out_max: int) -> int:
        if in_max == in_min:
            raise ValueError("Giriş aralığı sıfır olamaz")

        return int((value - in_min) * (out_max - out_min) /
                (in_max - in_min) + out_min)
#endregion

def main(args=None):
    """
    Start the steer node.
    """
    rclpy.init(args=args)

    node = SteerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_all_motors_sync()
        except Exception as ex:
            node.get_logger().error(f"Failed to stop motors on shutdown: {ex}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
