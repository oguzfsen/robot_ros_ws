
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message, serialize_message
from serial import Serial, SerialException
from robot_interfaces.msg import RobotStatus, IncomingMessage, Rtcm, Command, Mission


class TelemetryBridgeNode(Node):
    """Generic, whitelist-based ROS topic bridge over a binary serial link."""

    def __init__(self) -> None:
        super().__init__("telemetry_bridge_node", automatically_declare_parameters_from_overrides=True)

        self.serial_port = str(self._get_param("serial_port"))
        self.baudrate = int(self._get_param("baudrate"))
        self.serial_timeout = float(self._get_param("serial_timeout"))
        self.reconnect_delay = float(self._get_param("reconnect_delay_s"))
        self.write_idle_sleep = float(self._get_param("write_idle_sleep_s"))
        self.status_hz = float(self._get_param("status_hz"), 1.0)
        self.topic_status = str(self._get_param("topic_status"))
        self.topic_rtcm = str(self._get_param("topic_rtcm"))
        self.topic_command = str(self._get_param("topic_command"))
        self.topic_mission = str(self._get_param("topic_mission"))

        self.serial = Serial(port= self.serial_port, baudrate= self.baudrate, timeout=self.serial_timeout, write_timeout=self.serial_timeout)
        self.connected = True
        self.sending_threat = threading.Lock
        
        self.create_subscription(RobotStatus, self.topic_status, self.send_status, 10)
        self.rtcm_publisher = self.create_publisher(Rtcm, self.topic_rtcm)
        self.command_publisher = self.create_publisher(Command, self.topic_command)
        self.mission_publisher = self.create_publisher(Mission, self.topic_mission)
        self.save_mission_client = self.create_client(Mission, "/database/mission_set")
        self.last_request_id = 0


        self.listen_threat = threading.Thread(target=self.listen_message, daemon=True)
        self.listen_threat.start()

        self.get_logger().info("Telemetry up and running")


    def send_status(self, robot_status: RobotStatus) -> None:
        with self.sending_threat:
            robot_status.last_request_id = self.last_request_id
            buffer = serialize_message(robot_status)
        
        self.serial.write(buffer)


    def listen_message(self) -> None:
        buffer = self.serial.read_all()
        incoming_message = deserialize_message(buffer, IncomingMessage)
        msg_type = incoming_message.msg_type
        payload = incoming_message.payload

        if msg_type == "rtcm":
            self.rtcm_handler(payload)
        elif msg_type == "command":
            self.command_handler(payload)
        elif msg_type == "mission":
            self.mission_handler(payload)


    def rtcm_handler(self, payload: bytes) -> None:
        rtcm = deserialize_message(payload, Rtcm)
        self.rtcm_publisher.publish(rtcm)


    def command_handler(self, payload: bytes) -> None:
        command = deserialize_message(payload, Command)
        self.command_publisher.publish(command)


    def mission_handler(self, payload:bytes) -> None:
        mission = deserialize_message(payload, Mission)
        self.save_mission_to_db(mission)
    

    def save_mission_to_db(self):
        mission = ...
        self.save_mission_client.call_async(mission)
    

    def _get_param(self, name: str, default_value=None):
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

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
