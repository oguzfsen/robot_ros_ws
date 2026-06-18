import rclpy
from rclpy.node import Node
import serial
import re
from robot_interfaces.msg import RcInput
from typing import Optional
from rclpy.parameter import Parameter


class RCNode(Node):
    
    FIELD_PATTERN = re.compile(r"([a-z]+)=(-?\d+)")
    REQUIRED_FIELDS = {
        "rc": "rc",
        "crap": "crap",
        "thr": "throttle",
        "str": "steering",
        "mode": "mode",
        "aux": "aux",
    }
                              
    def __init__(self) -> None:
        super().__init__("rc_node", automatically_declare_parameters_from_overrides=True)

        self.serial_port = str(self._get_param("serial_port", "/dev/ttyUSB0"))
        self.serial_baud = int(self._get_param("serial_baud", 115200))        
        self.serial_timeout = float(self._get_param("serial_timeout", 0.1))        
        self.read_hz = float(self._get_param("read_hz", 10.0))
        self.last_msg = None
        
        self.publisher = self.create_publisher(RcInput, "/rc/input", 10)
        self.read_timer = self.create_timer(1 / self.read_hz, self.read_serial)

        self.serial: Optional[serial.Serial] = None
        self.open_serial()

        self.get_logger().info("Robot RC started.")


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

        
    
    def open_serial(self) -> None:
        """Open the serial port without crashing the node on failure."""
        try:
            self.serial = serial.Serial(
                self.serial_port,
                self.serial_baud,
                timeout=self.serial_timeout,
            )
            self.get_logger().info(
                f"Opened RC serial port {self.serial_port} @ {self.serial_baud}"
            )
        except Exception as exc:
            self.serial = None
            self.get_logger().error(
                f"Unable to open RC serial port {self.serial_port}: {exc}"
            )

    def ensure_serial(self) -> bool:
        """Reconnect in the background if the serial link is down."""
        if self.serial is not None and self.serial.is_open:
            return True
        self.open_serial()
        return self.serial is not None and self.serial.is_open
    
    def read_serial(self) -> None:
        if not self.ensure_serial():
            self.publish_disconnected()
            return

        try:
            raw = self.serial.readline()
        except Exception as exc:
            self.get_logger().error(f"RC serial read error: {exc}")
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            self.publish_disconnected()
            return

        if not raw:
            return

        line = raw.decode(errors="ignore").strip()
        if not line:
            return

        values = self.parse_line(line)
        if values is None:
            self.get_logger().warning(f"Ignoring unrecognized RC line: {line}")
            return
        else:
            msg = self.build_msg(values=values, raw_line=line)
            self.last_msg = msg
            self.publisher.publish(msg)



    def publish_disconnected(self) -> None:
        msg = self.build_msg(
            values={
                "rc": 0,
                "crap": 0,
                "throttle": 0,
                "steering": 0,
                "mode": 0,
                "aux": 0,
            },
            raw_line="",
        )
        
        self.publisher.publish(msg)
    

    def parse_line(self, line: str) -> Optional[dict]:
        """Parse Arduino text format into numeric RC channels."""
        parsed_fields = {
            key: int(value) for key, value in self.FIELD_PATTERN.findall(line)
        }
        if not all(field in parsed_fields for field in self.REQUIRED_FIELDS):
            return None

        return {
            target_key: parsed_fields[source_key]
            for source_key, target_key in self.REQUIRED_FIELDS.items()
        }

    def build_msg(self, values: dict, raw_line: str) -> RcInput:
        msg = RcInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.raw_line = raw_line
        msg.connected = bool(values["rc"])
        msg.crab = values["crap"]
        msg.throttle = values["throttle"]
        msg.steering = values["steering"]
        msg.ignition = values["mode"]
        msg.reverse = values["aux"]
        return msg

    def _get_param(self, name: str, default_value):
        """Read a parameter, declaring a default only when needed."""
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default_value).value

    def destroy_node(self) -> bool:
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
