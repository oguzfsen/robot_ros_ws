import math
import struct
import threading
import time
from typing import Callable, Optional

import serial


class GnssController:
    """
    ROS bagimsiz ve sade GNSS seri haberlesme sinifi.

    Sorumluluklari:
    - cihaza seri porttan baglanmak
    - karisik UBX + RTCM akisini parse etmek
    - station/base veya rover olarak cihaz ayari yollamak
    - son konum/fix bilgisini elde tutmak
    - station tarafinda RTCM callback cagirabilmek
    - rover tarafinda gelen RTCM verisini cihaza yazabilmek
    """

    UBX_SYNC = b"\xB5\x62"
    RTCM_PREAMBLE = 0xD3
    CRC24Q_POLY = 0x1864CFB

    KEY_CFG_USBINPROT_UBX = 0x10770001
    KEY_CFG_USBINPROT_NMEA = 0x10770002
    KEY_CFG_USBINPROT_RTCM3X = 0x10770004
    KEY_CFG_USBOUTPROT_UBX = 0x10780001
    KEY_CFG_USBOUTPROT_NMEA = 0x10780002
    KEY_CFG_USBOUTPROT_RTCM3X = 0x10780004
    KEY_CFG_RATE_MEAS = 0x30210001
    KEY_CFG_RATE_NAV = 0x30210002
    KEY_CFG_RATE_TIMEREF = 0x20210003
    KEY_CFG_TMODE_MODE = 0x20030001
    KEY_CFG_TMODE_POS_TYPE = 0x20030002
    KEY_CFG_TMODE_LAT = 0x40030009
    KEY_CFG_TMODE_LON = 0x4003000A
    KEY_CFG_TMODE_HEIGHT = 0x4003000B
    KEY_CFG_TMODE_LAT_HP = 0x2003000C
    KEY_CFG_TMODE_LON_HP = 0x2003000D
    KEY_CFG_TMODE_HEIGHT_HP = 0x2003000E
    KEY_CFG_TMODE_FIXED_POS_ACC = 0x4003000F
    KEY_CFG_MSGOUT_UBX_NAV_PVT_USB = 0x20910009
    KEY_CFG_MSGOUT_UBX_NAV_SAT_USB = 0x20910018

    KEY_CFG_MSGOUT_RTCM_1005_USB = 0x209102C0
    KEY_CFG_MSGOUT_RTCM_1074_USB = 0x20910361
    KEY_CFG_MSGOUT_RTCM_1084_USB = 0x20910366
    KEY_CFG_MSGOUT_RTCM_1094_USB = 0x2091036B
    KEY_CFG_MSGOUT_RTCM_1124_USB = 0x20910370
    KEY_CFG_MSGOUT_RTCM_1230_USB = 0x20910306

    RTCM_MESSAGE_KEYS = {
        1005: KEY_CFG_MSGOUT_RTCM_1005_USB,
        1074: KEY_CFG_MSGOUT_RTCM_1074_USB,
        1084: KEY_CFG_MSGOUT_RTCM_1084_USB,
        1094: KEY_CFG_MSGOUT_RTCM_1094_USB,
        1124: KEY_CFG_MSGOUT_RTCM_1124_USB,
        1230: KEY_CFG_MSGOUT_RTCM_1230_USB,
    }

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        serial_timeout_s: float = 0.2,
        measurement_rate_hz: float = 1.0,
    ):
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout_s = serial_timeout_s
        self.measurement_rate_hz = measurement_rate_hz

        self.serial_conn = None
        self._running = False
        self._read_thread = None
        self._write_lock = threading.Lock()

        self.device_mode = None

        self.station_lat = 38.624275938
        self.station_lon = 27.045235242
        self.station_alt = 12.0284
        self.station_accuracy_m = 2.675

        self.latest_latitude = 0.0
        self.latest_longitude = 0.0
        self.latest_altitude = 0.0
        self.latest_accuracy_m = 999.0
        self.latest_vertical_accuracy_m = 999.0
        self.latest_satellite_count = 0
        self.latest_gnss_fix_type = "No Fix"
        self.latest_ground_speed_m_s = 0.0
        self.latest_heading_deg = 0.0
        self.latest_speed_accuracy_m_s = 999.0
        self.latest_heading_accuracy_deg = 999.0
        self.latest_pdop = 99.99
        self.latest_fix_ok = False
        self.latest_differential_solution = False

        self.last_ack = None
        self.last_nak = None
        self.last_mon_ver = []

        self._ack_event = threading.Event()
        self._nak_event = threading.Event()
        self._rtcm_seen_event = threading.Event()
        self._nav_pvt_seen_event = threading.Event()

        self.rtcm_callback: Optional[Callable[[bytes], None]] = None
        self.rtcm_rates = {
            1005: 5.0,
            1074: 1.0,
            1084: 1.0,
            1094: 1.0,
            1124: 1.0,
            1230: 5.0,
        }
        self.rtcm_last_sent = {msg_id: 0.0 for msg_id in self.rtcm_rates}

    def connect(self) -> bool:
        if self.serial_conn and self.serial_conn.is_open:
            return True

        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.serial_timeout_s,
            )
            self._running = True
            self._read_thread = threading.Thread(
                target=self._listen_serial_loop,
                daemon=True,
            )
            self._read_thread.start()

            time.sleep(0.1)
            self._poll_ubx_message(0x0A, 0x04)

            print(
                f"[GNSS] {self.port} portuna {self.baudrate} baudrate ile baglanildi."
            )
            return True
        except Exception as exc:
            print(f"[GNSS HATA] Baglanti acilamadi: {exc}")
            self.serial_conn = None
            return False

    def disconnect(self):
        self._running = False

        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)

        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass

        self.serial_conn = None
        print(f"[GNSS] {self.port} baglantisi kapatildi.")

    def set_absolute_location(
        self,
        lat: float,
        lon: float,
        alt: float,
        accuracy_m: float = 2.675,
    ):
        self.station_lat = float(lat)
        self.station_lon = float(lon)
        self.station_alt = float(alt)
        self.station_accuracy_m = float(accuracy_m)

        print(
            "[GNSS] Sabit koordinatlar guncellendi. "
            f"Lat: {self.station_lat}, Lon: {self.station_lon}, Alt: {self.station_alt}"
        )

    def get_absolute_location(self) -> dict:
        return {
            "latitude": self.station_lat,
            "longitude": self.station_lon,
            "altitude": self.station_alt,
            "accuracy_m": self.station_accuracy_m,
        }

    def configure_station(self) -> bool:
        self.device_mode = "station"
        return self._configure_with_payload(
            self._build_station_valset_payload(),
            timeout_s=6.0,
            start_message="[GNSS] Station/base konfigurasyonu gonderiliyor...",
        )

    def configure_rover(self) -> bool:
        self.device_mode = "rover"
        return self._configure_with_payload(
            self._build_rover_valset_payload(),
            timeout_s=6.0,
            start_message="[GNSS] Rover konfigurasyonu gonderiliyor...",
        )

    def get_position(self) -> dict:
        return {
            "latitude": self.latest_latitude,
            "longitude": self.latest_longitude,
            "altitude": self.latest_altitude,
            "accuracy_m": self.latest_accuracy_m,
            "vertical_accuracy_m": self.latest_vertical_accuracy_m,
            "satellite_count": self.latest_satellite_count,
            "fix_type": self.latest_gnss_fix_type,
            "ground_speed_m_s": self.latest_ground_speed_m_s,
            "heading_deg": self.latest_heading_deg,
            "speed_accuracy_m_s": self.latest_speed_accuracy_m_s,
            "heading_accuracy_deg": self.latest_heading_accuracy_deg,
            "pdop": self.latest_pdop,
            "fix_ok": self.latest_fix_ok,
            "differential_solution": self.latest_differential_solution,
        }

    def get_rover_position(self) -> dict:
        return self.get_position()

    def send_rtcm(self, rtcm_frame: bytes):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        with self._write_lock:
            self.serial_conn.write(rtcm_frame)
            self.serial_conn.flush()

    def set_rtcm_callback(self, callback_func):
        self.rtcm_callback = callback_func

    def _configure_with_payload(
        self,
        payload: bytes,
        timeout_s: float,
        start_message: str,
    ) -> bool:
        if not self.serial_conn or not self.serial_conn.is_open:
            print("[GNSS HATA] Cihaz konfigure edilemedi: Seri port acik degil.")
            return False

        self._clear_config_events()
        print(start_message)
        self._send_ubx(0x06, 0x8A, payload)

        success = self._wait_for_config_result(timeout_s=timeout_s)
        if success:
            self._poll_ubx_message(0x01, 0x07)
            self._poll_ubx_message(0x01, 0x35)
        return success

    def _listen_serial_loop(self):
        buffer = bytearray()
        last_nav_sat_poll = 0.0

        while self._running:
            try:
                waiting = self.serial_conn.in_waiting if self.serial_conn else 0

                if waiting > 0:
                    chunk = self.serial_conn.read(waiting)
                    if chunk:
                        buffer.extend(chunk)
                else:
                    if time.time() - last_nav_sat_poll >= 2.0:
                        self._poll_ubx_message(0x01, 0x35)
                        last_nav_sat_poll = time.time()
                    time.sleep(0.01)
                    continue

                self._consume_serial_buffer(buffer)

            except Exception as exc:
                if not self._running:
                    break
                print(f"[GNSS DONGU HATASI] {exc}")
                time.sleep(0.2)

    def _consume_serial_buffer(self, buffer: bytearray):
        while len(buffer) >= 2:
            if buffer[0] == self.UBX_SYNC[0] and buffer[1] == self.UBX_SYNC[1]:
                if len(buffer) < 6:
                    return

                payload_len = struct.unpack("<H", buffer[4:6])[0]
                total_len = 6 + payload_len + 2
                if len(buffer) < total_len:
                    return

                frame = bytes(buffer[:total_len])
                if not self._is_valid_ubx_frame(frame):
                    buffer.pop(0)
                    continue

                del buffer[:total_len]
                self._parse_ubx_frame(frame)
                continue

            if buffer[0] == self.RTCM_PREAMBLE:
                if len(buffer) < 3:
                    return

                payload_len = ((buffer[1] & 0x03) << 8) | buffer[2]
                total_len = 3 + payload_len + 3
                if len(buffer) < total_len:
                    return

                frame = bytes(buffer[:total_len])
                if not self._is_valid_rtcm_frame(frame):
                    buffer.pop(0)
                    continue

                del buffer[:total_len]
                self._handle_incoming_rtcm(frame)
                continue

            next_sync_positions = [
                idx
                for idx in (
                    buffer.find(self.UBX_SYNC),
                    buffer.find(bytes([self.RTCM_PREAMBLE])),
                )
                if idx > 0
            ]

            if next_sync_positions:
                del buffer[: min(next_sync_positions)]
            else:
                buffer.pop(0)

    def _parse_ubx_frame(self, frame: bytes):
        msg_class = frame[2]
        msg_id = frame[3]
        payload = frame[6:-2]

        if msg_class == 0x01 and msg_id == 0x07:
            self._parse_nav_pvt(payload)
            return

        if msg_class == 0x01 and msg_id == 0x35:
            self._parse_nav_sat(payload)
            return

        if msg_class == 0x05 and msg_id == 0x01:
            self._parse_ack_ack(payload)
            return

        if msg_class == 0x05 and msg_id == 0x00:
            self._parse_ack_nak(payload)
            return

        if msg_class == 0x0A and msg_id == 0x04:
            self._parse_mon_ver(payload)

    def _parse_nav_pvt(self, payload: bytes):
        if len(payload) < 44:
            return

        fix_type = payload[20]
        flags = payload[21]
        num_sv = payload[23]
        lon = struct.unpack_from("<i", payload, 24)[0] / 1e7
        lat = struct.unpack_from("<i", payload, 28)[0] / 1e7
        altitude_m = struct.unpack_from("<i", payload, 36)[0] / 1000.0
        horizontal_accuracy_m = struct.unpack_from("<I", payload, 40)[0] / 1000.0

        self.latest_latitude = lat
        self.latest_longitude = lon
        self.latest_altitude = altitude_m
        self.latest_accuracy_m = horizontal_accuracy_m
        self.latest_satellite_count = num_sv
        self.latest_gnss_fix_type = self._decode_fix_type(fix_type, flags)
        self.latest_fix_ok = bool(flags & 0x01)
        self.latest_differential_solution = bool(flags & 0x02)

        if len(payload) >= 78:
            self.latest_vertical_accuracy_m = (
                struct.unpack_from("<I", payload, 44)[0] / 1000.0
            )
            self.latest_ground_speed_m_s = (
                struct.unpack_from("<i", payload, 60)[0] / 1000.0
            )
            self.latest_heading_deg = (
                struct.unpack_from("<i", payload, 64)[0] / 100000.0
            )
            self.latest_speed_accuracy_m_s = (
                struct.unpack_from("<I", payload, 68)[0] / 1000.0
            )
            self.latest_heading_accuracy_deg = (
                struct.unpack_from("<I", payload, 72)[0] / 100000.0
            )
            self.latest_pdop = struct.unpack_from("<H", payload, 76)[0] / 100.0

        self._nav_pvt_seen_event.set()

    def _parse_nav_sat(self, payload: bytes):
        if len(payload) < 6:
            return

    def _parse_ack_ack(self, payload: bytes):
        if len(payload) < 2:
            return

        self.last_ack = (payload[0], payload[1])
        if self.last_ack == (0x06, 0x8A):
            self._ack_event.set()

    def _parse_ack_nak(self, payload: bytes):
        if len(payload) < 2:
            return

        self.last_nak = (payload[0], payload[1])
        if self.last_nak == (0x06, 0x8A):
            self._nak_event.set()

    def _parse_mon_ver(self, payload: bytes):
        if len(payload) < 40:
            return

        lines = []

        sw_version = payload[0:30].rstrip(b"\x00").decode("ascii", errors="ignore")
        hw_version = payload[30:40].rstrip(b"\x00").decode("ascii", errors="ignore")

        if sw_version:
            lines.append(f"SW={sw_version}")
        if hw_version:
            lines.append(f"HW={hw_version}")

        for offset in range(40, len(payload), 30):
            ext = payload[offset : offset + 30].rstrip(b"\x00").decode(
                "ascii",
                errors="ignore",
            )
            if ext:
                lines.append(ext)

        self.last_mon_ver = lines

    def _handle_incoming_rtcm(self, frame: bytes):
        self._rtcm_seen_event.set()

        if self.device_mode != "station":
            return

        msg_type = self._extract_rtcm_message_type(frame)
        if msg_type is None:
            return

        period_s = self.rtcm_rates.get(msg_type)
        now = time.time()

        if period_s is not None:
            last_sent = self.rtcm_last_sent.get(msg_type, 0.0)
            if now - last_sent < period_s:
                return
            self.rtcm_last_sent[msg_type] = now

        if self.rtcm_callback is None:
            return

        try:
            self.rtcm_callback(frame)
        except Exception as exc:
            print(f"[GNSS RTCM CALLBACK HATASI] {exc}")

    def _build_station_valset_payload(self) -> bytes:
        lat_main, lat_hp = self._split_deg_value(self.station_lat)
        lon_main, lon_hp = self._split_deg_value(self.station_lon)
        height_main, height_hp = self._split_height_value(self.station_alt)
        accuracy_01mm = max(1, int(round(self.station_accuracy_m * 10000.0)))

        items = self._build_common_usb_items(enable_rtcm_output=True)
        items.extend(
            [
                (self.KEY_CFG_TMODE_MODE, 2, "U1"),
                (self.KEY_CFG_TMODE_POS_TYPE, 1, "U1"),
                (self.KEY_CFG_TMODE_LAT, lat_main, "I4"),
                (self.KEY_CFG_TMODE_LON, lon_main, "I4"),
                (self.KEY_CFG_TMODE_HEIGHT, height_main, "I4"),
                (self.KEY_CFG_TMODE_LAT_HP, lat_hp, "I1"),
                (self.KEY_CFG_TMODE_LON_HP, lon_hp, "I1"),
                (self.KEY_CFG_TMODE_HEIGHT_HP, height_hp, "I1"),
                (self.KEY_CFG_TMODE_FIXED_POS_ACC, accuracy_01mm, "U4"),
            ]
        )

        for key_id in self.RTCM_MESSAGE_KEYS.values():
            items.append((key_id, 1, "U1"))

        return self._build_valset_payload(items)

    def _build_rover_valset_payload(self) -> bytes:
        items = self._build_common_usb_items(enable_rtcm_output=False)
        items.extend(
            [
                (self.KEY_CFG_USBINPROT_RTCM3X, 1, "U1"),
                (self.KEY_CFG_TMODE_MODE, 0, "U1"),
            ]
        )
        return self._build_valset_payload(items)

    def _build_common_usb_items(self, enable_rtcm_output: bool) -> list:
        measurement_period_ms = self._get_measurement_period_ms()

        return [
            (self.KEY_CFG_USBINPROT_UBX, 1, "U1"),
            (self.KEY_CFG_USBINPROT_NMEA, 0, "U1"),
            (self.KEY_CFG_USBOUTPROT_UBX, 1, "U1"),
            (self.KEY_CFG_USBOUTPROT_NMEA, 0, "U1"),
            (self.KEY_CFG_USBOUTPROT_RTCM3X, 1 if enable_rtcm_output else 0, "U1"),
            (self.KEY_CFG_RATE_MEAS, measurement_period_ms, "U2"),
            (self.KEY_CFG_RATE_NAV, 1, "U2"),
            (self.KEY_CFG_RATE_TIMEREF, 1, "U1"),
            (self.KEY_CFG_MSGOUT_UBX_NAV_PVT_USB, 1, "U1"),
            (self.KEY_CFG_MSGOUT_UBX_NAV_SAT_USB, 1, "U1"),
        ]

    def _build_valset_payload(self, items: list, layers: int = 0x01) -> bytes:
        payload = bytearray([0x00, layers, 0x00, 0x00])

        for key_id, value, value_type in items:
            payload.extend(struct.pack("<I", key_id))
            payload.extend(self._encode_config_value(value, value_type))

        return bytes(payload)

    def _encode_config_value(self, value, value_type: str) -> bytes:
        if value_type == "U1":
            return struct.pack("<B", int(value))
        if value_type == "I1":
            return struct.pack("<b", int(value))
        if value_type == "U2":
            return struct.pack("<H", int(value))
        if value_type == "I2":
            return struct.pack("<h", int(value))
        if value_type == "U4":
            return struct.pack("<I", int(value))
        if value_type == "I4":
            return struct.pack("<i", int(value))
        raise ValueError(f"Desteklenmeyen UBX config tipi: {value_type}")

    def _get_measurement_period_ms(self) -> int:
        if self.measurement_rate_hz <= 0:
            return 1000

        period_ms = int(round(1000.0 / self.measurement_rate_hz))
        return max(200, period_ms)

    def _split_deg_value(self, deg_value: float) -> tuple:
        total_nano_deg = int(round(deg_value * 1_000_000_000.0))
        coarse = int(math.trunc(total_nano_deg / 100))
        high_precision = total_nano_deg - (coarse * 100)
        return coarse, self._clamp_i1(high_precision)

    def _split_height_value(self, height_m: float) -> tuple:
        total_01mm = int(round(height_m * 10000.0))
        coarse_cm = int(math.trunc(total_01mm / 100))
        high_precision = total_01mm - (coarse_cm * 100)
        return coarse_cm, self._clamp_i1(high_precision)

    def _clamp_i1(self, value: int) -> int:
        return max(-99, min(99, int(value)))

    def _wait_for_config_result(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            if self._nak_event.is_set():
                print("[GNSS HATA] UBX-CFG-VALSET NAK aldi.")
                return False

            if self._ack_event.is_set():
                print("[GNSS] UBX-CFG-VALSET ACK aldi.")
                return True

            if self.device_mode == "station" and self._rtcm_seen_event.is_set():
                print("[GNSS] RTCM akisi basladi, konfigurasyon basarili kabul edildi.")
                return True

            if self.device_mode == "rover" and self._nav_pvt_seen_event.is_set():
                print("[GNSS] NAV-PVT akisi goruldu, konfigurasyon basarili kabul edildi.")
                return True

            time.sleep(0.05)

        print("[GNSS] Konfigurasyon bekleme suresi doldu.")
        return False

    def _clear_config_events(self):
        self.last_ack = None
        self.last_nak = None
        self._ack_event.clear()
        self._nak_event.clear()
        self._rtcm_seen_event.clear()
        self._nav_pvt_seen_event.clear()

    def _send_ubx(self, msg_class: int, msg_id: int, payload: bytes = b""):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        frame = self._wrap_ubx_frame(msg_class, msg_id, payload)
        with self._write_lock:
            self.serial_conn.write(frame)
            self.serial_conn.flush()

    def _poll_ubx_message(self, msg_class: int, msg_id: int):
        try:
            self._send_ubx(msg_class, msg_id, b"")
        except Exception:
            pass

    def _wrap_ubx_frame(self, msg_class: int, msg_id: int, payload: bytes) -> bytes:
        header = struct.pack("<BBH", msg_class, msg_id, len(payload))
        ck_a, ck_b = self._calculate_ubx_checksum(header + payload)
        return self.UBX_SYNC + header + payload + bytes([ck_a, ck_b])

    def _calculate_ubx_checksum(self, content: bytes) -> tuple:
        ck_a = 0
        ck_b = 0

        for byte in content:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        return ck_a, ck_b

    def _is_valid_ubx_frame(self, frame: bytes) -> bool:
        if len(frame) < 8:
            return False

        ck_a, ck_b = self._calculate_ubx_checksum(frame[2:-2])
        return frame[-2] == ck_a and frame[-1] == ck_b

    def _is_valid_rtcm_frame(self, frame: bytes) -> bool:
        if len(frame) < 6 or frame[0] != self.RTCM_PREAMBLE:
            return False

        expected_crc = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
        actual_crc = self._crc24q(frame[:-3])
        return expected_crc == actual_crc

    def _crc24q(self, data: bytes) -> int:
        crc = 0
        for byte in data:
            crc ^= byte << 16
            for _ in range(8):
                crc <<= 1
                if crc & 0x1000000:
                    crc ^= self.CRC24Q_POLY
        return crc & 0xFFFFFF

    def _extract_rtcm_message_type(self, frame: bytes) -> Optional[int]:
        if len(frame) < 5:
            return None
        return (frame[3] << 4) | (frame[4] >> 4)

    def _decode_fix_type(self, fix_type: int, flags: int) -> str:
        carrier_solution = (flags >> 6) & 0x03
        diff_solution = bool(flags & 0x02)

        if carrier_solution == 2:
            return "RTK Fixed"
        if carrier_solution == 1:
            return "RTK Float"
        if fix_type == 5:
            return "Time"
        if fix_type == 4:
            return "GNSS+DR"
        if fix_type == 3 and diff_solution:
            return "DGNSS"
        if fix_type == 3:
            return "3D Fix"
        if fix_type == 2:
            return "2D Fix"
        if fix_type == 1:
            return "DR Only"
        return "No Fix"
