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


def test_writes_comma_separated_log_and_creates_directory(tmp_path, monkeypatch):
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
    assert path == log_directory / "compass_log_2026-09-07_12-34-56.csv"
    assert path.read_text(encoding="utf-8") == (
        "timestamp,heading_deg,fix_ok\n"
        "2026-09-07T15:34:56.123+03:00,123.5,1\n"
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

    assert first_path.name == "compass_log_2026-09-07_12-34-56.csv"
    assert second_path.name == "compass_log_2026-09-07_12-34-56_01.csv"


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
    logger.log(compass, gnss, average_steering=2.5, rc_reverse=True, rc_throttle=35)
    logger.close()

    path = tmp_path / "compass_log_2026-09-07_12-34-56.csv"
    assert path.read_text(encoding="utf-8").splitlines() == [
        ",".join(drive_log_writer.COMPASS_GNSS_LOG_COLUMNS),
        "2026-09-07T12:34:56.000+00:00,1.0,2.0,3.0,45.0"
        ",47.0,0.5,1.25,1,2.5,True,35",
    ]


def test_autonomous_logger_records_time_and_separates_sessions(tmp_path, monkeypatch):
    monkeypatch.setattr(drive_log_writer, 'datetime', FixedDatetime)
    monkeypatch.setattr(drive_log_writer.time, 'monotonic', lambda: 10.0)
    logger = drive_log_writer.AutonomousLogger(tmp_path, RecordingLogger())
    row = dict.fromkeys(drive_log_writer.AUTONOMOUS_LOG_COLUMNS)
    row.update(event='update', mission_id=42, phase='on_line', throttle=35)
    logger.start(42)
    first_path = logger._writer.path
    monkeypatch.setattr(drive_log_writer.time, 'monotonic', lambda: 10.1)
    logger.log(row)
    logger.start(42)
    second_path = logger._writer.path
    logger.log(row)
    logger.close()
    logger.log(row)

    with first_path.open() as stream:
        records = list(drive_log_writer.csv.DictReader(stream, delimiter=','))
    assert len(records) == 1
    assert records[0]['timestamp'] == '2026-09-07T12:34:56.000+00:00'
    assert float(records[0]['elapsed_s']) == pytest.approx(0.1)
    assert records[0]['phase'] == 'on_line' and records[0]['throttle'] == '35'
    assert first_path.name == 'autonomous_log_42_2026-09-07_12-34-56.csv'
    assert second_path.name == 'autonomous_log_42_2026-09-07_12-34-56_01.csv'
    assert len(second_path.read_text().splitlines()) == 2


def test_autonomous_logger_start_failure_is_reported_without_raising(tmp_path):
    blocked_directory = tmp_path / 'file'
    blocked_directory.write_text('not a directory')
    messages = RecordingLogger()
    logger = drive_log_writer.AutonomousLogger(blocked_directory, messages)
    logger.start(42)
    logger.log({})
    logger.close()
    assert not logger._writer.is_open
    assert len(messages.messages) == 1
    assert messages.messages[0][0] == 'error'


def test_csv_preserves_commas_quotes_and_newlines_in_errors(tmp_path):
    writer = DriveLogWriter(('event', 'error'), tmp_path)
    path = writer.start()
    row = {'event': 'error', 'error': 'Hata: gnss=42, pusula=67; "tekrar"\nyeni satir'}
    writer.write(row)
    writer.close()
    with path.open(newline='') as stream:
        assert list(drive_log_writer.csv.DictReader(stream)) == [row]
