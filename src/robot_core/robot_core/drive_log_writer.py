#!/usr/bin/env python3

import csv
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Mapping, Sequence, TextIO


COMPASS_GNSS_LOG_COLUMNS = (
    "timestamp",
    "compass_x",
    "compass_y",
    "compass_z",
    "compass_heading_deg",
    "gnss_heading_deg",
    "gnss_heading_accuracy_deg",
    "gnss_ground_speed_m_s",
    "gnss_fix_ok",
    "average_steering",
    "rc_reverse",
    "rc_throttle"
)


class DriveLogWriter:
    """Thread-safe, tab-separated log writer for robot drive tests."""

    def __init__(
        self,
        columns: Sequence[str],
        log_directory: Path,
        file_prefix: str = "drive_log",
    ) -> None:
        if not columns:
            raise ValueError("At least one log column is required")
        if len(columns) != len(set(columns)):
            raise ValueError("Log column names must be unique")

        self.columns = tuple(columns)
        self.log_directory = Path(log_directory).expanduser()
        self.file_prefix = file_prefix

        self._lock = Lock()
        self._stream: TextIO | None = None
        self._writer: csv.DictWriter | None = None
        self._path: Path | None = None

    @property
    def path(self) -> Path | None:
        with self._lock:
            return self._path

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._stream is not None

    def start(self) -> Path:
        """Create a new log file and write its header."""
        with self._lock:
            if self._stream is not None:
                raise RuntimeError("A drive log file is already open")

            self.log_directory.mkdir(parents=True, exist_ok=True)
            timestamp = (
                datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
            )

            stream, path = self._open_unique_file(timestamp)
            writer = csv.DictWriter(
                stream,
                fieldnames=self.columns,
                delimiter="\t",
                lineterminator="\n",
                extrasaction="raise",
            )
            writer.writeheader()
            stream.flush()

            self._stream = stream
            self._writer = writer
            self._path = path
            return path

    def write(self, values: Mapping[str, object]) -> bool:
        """Write and flush one record. Return False when no log is open."""
        with self._lock:
            if self._stream is None or self._writer is None:
                return False

            missing_columns = set(self.columns).difference(values)
            if missing_columns:
                missing = ", ".join(sorted(missing_columns))
                raise ValueError(f"Missing log columns: {missing}")

            self._writer.writerow(values)
            self._stream.flush()
            return True

    def close(self) -> None:
        with self._lock:
            if self._stream is None:
                return

            stream = self._stream
            self._stream = None
            self._writer = None
            try:
                stream.flush()
            finally:
                stream.close()

    def _open_unique_file(self, timestamp: str) -> tuple[TextIO, Path]:
        for sequence in range(1000):
            suffix = "" if sequence == 0 else f"_{sequence:02d}"
            path = self.log_directory / (
                f"{self.file_prefix}_{timestamp}{suffix}.txt"
            )
            try:
                stream = path.open(
                    "x",
                    encoding="utf-8",
                    newline="",
                    buffering=1,
                )
                return stream, path
            except FileExistsError:
                continue

        raise FileExistsError(
            f"Could not create a unique log file in {self.log_directory}"
        )


class CompassGnssLogger:
    """Manage Compass/GNSS log sessions without coupling them to a ROS node."""

    def __init__(self, log_directory: Path, logger) -> None:
        self._logger = logger
        self._writer = DriveLogWriter(
            columns=COMPASS_GNSS_LOG_COLUMNS,
            log_directory=log_directory,
            file_prefix="compass_log",
        )
        self._missing_data_warning_sent = False

    def start(self) -> None:
        try:
            path = self._writer.start()
        except (OSError, RuntimeError) as exc:
            self._logger.error(f"Compass/GNSS log start failed: {exc}")
            return

        self._missing_data_warning_sent = False
        self._logger.info(f"Compass/GNSS logging started. file={path}")

    def log(self, compass, gnss, average_steering: float, rc_reverse: bool, rc_throttle: int) -> None:
        if not self._writer.is_open:
            return

        if compass is None or gnss is None:
            if not self._missing_data_warning_sent:
                self._logger.warning(
                    "Waiting for both compass and GNSS data before logging"
                )
                self._missing_data_warning_sent = True
            return

        try:
            self._writer.write(
                {
                    "timestamp": datetime.now().astimezone().isoformat(
                        timespec="milliseconds"
                    ),
                    "compass_x": float(compass.x),
                    "compass_y": float(compass.y),
                    "compass_z": float(compass.z),
                    "compass_heading_deg": float(compass.heading_deg),
                    "gnss_heading_deg": float(gnss.heading_deg),
                    "gnss_heading_accuracy_deg": float(
                        gnss.heading_accuracy_deg
                    ),
                    "gnss_ground_speed_m_s": float(gnss.ground_speed_m_s),
                    "gnss_fix_ok": int(gnss.fix_ok),
                    "average_steering": float(average_steering),
                    "rc_reverse": bool(rc_reverse),
                    "rc_throttle": int(rc_throttle)
                }
            )
        except (OSError, ValueError) as exc:
            self._logger.error(f"Compass/GNSS log write failed: {exc}")
            self.close()

    def close(self) -> None:
        if not self._writer.is_open:
            return

        path = self._writer.path
        try:
            self._writer.close()
        except OSError as exc:
            self._logger.error(f"Compass/GNSS log close failed: {exc}")
            return

        self._logger.info(f"Compass/GNSS log closed. file={path}")
