from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from robot_core import drive_log_writer
from robot_core.drive_log_writer import CompassGnssLogger, DriveLogWriter


class FixedDatetime(datetime):
    @classmethod
    def now(cls, tz=None):
        return cls(2026, 9, 7, 12, 34, 56, tzinfo=timezone.utc)

    def astimezone(self, tz=None):
        return self


class RecordingLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warning(self, message):
        self.messages.append(("warning", message))

    def error(self, message):
        self.messages.append(("error", message))


def test_writes_tab_separated_log_and_creates_directory(tmp_path, monkeypatch):
    monkeypatch.setattr(drive_log_writer, "datetime", FixedDatetime)
    log_directory = tmp_path / "nested" / "logs"
    writer = DriveLogWriter(
        columns=("timestamp", "heading_deg", "fix_ok"),
        log_directory=log_directory,
        file_prefix="compass_log",
    )

    path = writer.start()
    written = writer.write(
        {
            "timestamp": "2026-09-07T15:34:56.123+03:00",
            "heading_deg": 123.5,
            "fix_ok": 1,
        }
    )
    writer.close()

    assert written
    assert path == log_directory / "compass_log_2026-09-07_12-34-56.txt"
    assert path.read_text(encoding="utf-8") == (
        "timestamp\theading_deg\tfix_ok\n"
        "2026-09-07T15:34:56.123+03:00\t123.5\t1\n"
    )


def test_uses_sequence_suffix_instead_of_overwriting(tmp_path, monkeypatch):
    monkeypatch.setattr(drive_log_writer, "datetime", FixedDatetime)
    columns = ("value",)

    first_writer = DriveLogWriter(columns, tmp_path, "compass_log")
    first_path = first_writer.start()
    first_writer.close()

    second_writer = DriveLogWriter(columns, tmp_path, "compass_log")
    second_path = second_writer.start()
    second_writer.close()

    assert first_path.name == "compass_log_2026-09-07_12-34-56.txt"
    assert second_path.name == "compass_log_2026-09-07_12-34-56_01.txt"


def test_rejects_incomplete_rows(tmp_path):
    writer = DriveLogWriter(("first", "second"), tmp_path)
    writer.start()

    with pytest.raises(ValueError, match="Missing log columns: second"):
        writer.write({"first": 1})

    writer.close()


def test_write_returns_false_when_log_is_closed(tmp_path):
    writer = DriveLogWriter(("value",), tmp_path)

    assert not writer.write({"value": 1})


def test_compass_gnss_logger_writes_sensor_fields(tmp_path, monkeypatch):
    monkeypatch.setattr(drive_log_writer, "datetime", FixedDatetime)
    logger = CompassGnssLogger(tmp_path, RecordingLogger())
    compass = SimpleNamespace(x=1.0, y=2.0, z=3.0, heading_deg=45.0)
    gnss = SimpleNamespace(
        heading_deg=47.0,
        heading_accuracy_deg=0.5,
        ground_speed_m_s=1.25,
        fix_ok=True,
    )

    logger.start()
    logger.log(compass, gnss, average_steering=2.5)
    logger.close()

    path = tmp_path / "compass_log_2026-09-07_12-34-56.txt"
    assert path.read_text(encoding="utf-8").splitlines() == [
        "\t".join(drive_log_writer.COMPASS_GNSS_LOG_COLUMNS),
        "2026-09-07T12:34:56.000+00:00\t1.0\t2.0\t3.0\t45.0"
        "\t47.0\t0.5\t1.25\t1\t2.5",
    ]
