
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from serial import Serial, SerialException
from robot_interfaces.msg import RobotStatus, IncomingMessage, Rtcm, Command, Mission
from Tools import _get_param


class TelemetryBridgeNode(Node):
    """Generic, whitelist-based ROS topic bridge over a binary serial link."""

    def __init__(self) -> None:
        super().__init__("telemetry_bridge_node", automatically_declare_parameters_from_overrides=True)

        self.serial_port = str(_get_param(self, "serial_port", "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"))
        self.baudrate = int(_get_param(self, "baudrate", 460800))
        self.serial_timeout = float(_get_param(self, "serial_timeout", 0.1))
        self.reconnect_delay = float(_get_param(self, "reconnect_delay_s", 2.0))
        self.write_idle_sleep = float(_get_param(self, "write_idle_sleep_s", 0.01))
        self.status_hz = float(_get_param(self, "status_hz"), 1.0)
        self.topic_status = str(_get_param(self, "topic_status", "/robot/status"))
        self.topic_rtcm = str(_get_param(self, "topic_rtcm", "/station/rtcm"))
        self.topic_command = str(_get_param(self, "topic_command", "/user/command"))
        self.topic_mission = str(_get_param(self, "topic_mission", "/user/mission"))

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
    

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
