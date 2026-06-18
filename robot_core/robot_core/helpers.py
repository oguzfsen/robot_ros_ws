from enum import IntEnum
import secrets
import string
from datetime import datetime
from datetime import datetime as dt
from typing import Optional


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

