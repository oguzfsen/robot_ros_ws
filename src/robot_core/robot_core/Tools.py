from enum import IntEnum
import secrets
import string
from datetime import datetime
from datetime import datetime as dt
from typing import Optional

from rclpy.parameter import Parameter


class RobotState(IntEnum):
    IDLE = 1
    MANUAL = 2
    MISSION = 3
    EMERGENCY = 4
    MALFUNTION = 5


def create_id(length: int = 8) -> str:
    alphabet = string.ascii_uppercase + string.digits
    return "".join(secrets.choice(alphabet) for _ in range(length))


def time_to_str(time_obj: Optional[dt] = None) -> str:
    """Datetime nesnesini ISO string'e çevirir. Boşsa şu anki zamanı alır."""
    if time_obj is None:
        time_obj = dt.now()
    return time_obj.isoformat()

def str_to_time(time_str: str) -> dt:
    """ISO string'i tekrar datetime nesnesine çevirir."""
    return dt.fromisoformat(time_str)

def clamp_int(value: int, minimum: int, maximum: int) -> int:
    if minimum > maximum:
        raise ValueError("minimum cannot be greater than maximum")
    return min(max(value, minimum), maximum)

def _get_param(cls, name: str, default_value=None):
    """
    Generic ROS2 parameter reader.

    - If parameter exists -> returns plain Python value
    - If not exists -> declares with default and returns value
    - If a Parameter object is encountered -> unwraps it
    - Works for int, float, bool, str, list, etc.
    """

    try:
        if cls.has_parameter(name):
            param = cls.get_parameter(name)
        else:
            param = cls.declare_parameter(name, default_value)
    except Exception:
        param = cls.get_parameter(name)

    if not isinstance(param, Parameter):
        return param

    try:
        return param.value
    except Exception:
        pass

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
