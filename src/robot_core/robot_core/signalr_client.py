#!/usr/bin/env python3

import json
import queue
import threading
import time
from typing import Callable
from urllib.parse import urlparse

import requests


class SignalRLongPollingClient:
    """Minimal SignalR client that works with ASP.NET Core long polling."""

    RECORD_SEPARATOR = "\x1e"

    def __init__(
        self,
        hub_url: str,
        send_method_name: str,
        receive_method_name: str,
        reconnect_delay_s: float,
        poll_timeout_s: float,
        http_timeout_s: float,
        on_receive_message: Callable[[str, str], None],
        log_info: Callable[[str], None],
        log_warn: Callable[[str], None],
    ):
        self.hub_url = hub_url.rstrip("/")
        self.send_method_name = send_method_name
        self.receive_method_name = receive_method_name
        self.reconnect_delay_s = reconnect_delay_s
        self.poll_timeout_s = poll_timeout_s
        self.http_timeout_s = http_timeout_s
        self.on_receive_message = on_receive_message
        self.log_info = log_info
        self.log_warn = log_warn

        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._connection_thread = None
        self._sender_thread = None
        self._send_queue = queue.Queue(maxsize=20)
        self._connection_id = ""
        self._connection_token = ""
        self._handshake_ok = False
        self._is_connect_log_printed = False
        self._invoke_lock = threading.Lock()
        self._reconnect_requested = threading.Event()

    def start(self) -> None:
        if self._connection_thread and self._connection_thread.is_alive():
            return

        self._stop_event.clear()
        self._connection_thread = threading.Thread(
            target=self._connection_loop,
            daemon=True,
        )
        self._sender_thread = threading.Thread(
            target=self._sender_loop,
            daemon=True,
        )
        self._connection_thread.start()
        self._sender_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._connected_event.clear()
        self._reconnect_requested.set()

        try:
            self._send_queue.put_nowait(None)
        except queue.Full:
            pass

        self._close_connection()

        if self._connection_thread and self._connection_thread.is_alive():
            self._connection_thread.join(timeout=2.0)

        if self._sender_thread and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=2.0)

    def is_connected(self) -> bool:
        return self._connected_event.is_set() and self._handshake_ok

    def send(self, topic: str, json_text: str) -> None:
        item = (topic, json_text)

        if self._send_queue.full():
            try:
                self._send_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self._send_queue.put_nowait(item)
        except queue.Full:
            self.log_warn("SignalR gonderim kuyrugu dolu, en yeni mesaj atlandi.")

    def send_now(
        self,
        topic: str,
        json_text: str,
        wait_for_connection_s: float = 0.0,
    ) -> None:
        if self._stop_event.is_set():
            raise RuntimeError("SignalR istemcisi durduruldu.")

        if wait_for_connection_s > 0.0:
            connected = self._connected_event.wait(timeout=wait_for_connection_s)
        else:
            connected = self._connected_event.is_set()

        if not connected or not self._handshake_ok:
            raise RuntimeError("SignalR baglantisi hazir degil.")

        self._send_invocation(topic, json_text)

    def _connection_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._reconnect_requested.clear()
                self._open_connection()
                self._poll_loop()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.log_warn(f"SignalR baglantisi koptu: {exc}")
            finally:
                self._connected_event.clear()
                self._handshake_ok = False
                self._is_connect_log_printed = False
                self._close_connection()

            if not self._stop_event.is_set():
                time.sleep(self.reconnect_delay_s)

    def _sender_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                item = self._send_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if item is None:
                continue

            topic, json_text = item

            while not self._stop_event.is_set():
                try:
                    self.send_now(topic, json_text, wait_for_connection_s=0.5)
                    break
                except Exception as exc:
                    self.log_warn(f"SignalR mesaji gonderilemedi: {exc}")
                    time.sleep(1.0)

    def _open_connection(self) -> None:
        response = self._post_negotiate()
        response.raise_for_status()

        payload = response.json()
        if payload.get("error"):
            raise RuntimeError(f"negotiate hatasi: {payload['error']}")

        self._connection_id = str(payload.get("connectionId", "")).strip()
        self._connection_token = str(
            payload.get("connectionToken") or self._connection_id
        ).strip()

        if not self._connection_token:
            raise RuntimeError("SignalR connection token bos dondu.")

        self._send_raw_frame(self._build_handshake_frame())

    def _post_negotiate(self) -> requests.Response:
        negotiate_url = f"{self.hub_url}/negotiate"
        response = requests.post(
            negotiate_url,
            params={"negotiateVersion": 1},
            timeout=self.http_timeout_s,
            allow_redirects=False,
        )

        if response.status_code not in (301, 302, 307, 308):
            return response

        location = str(response.headers.get("Location", "")).strip()
        if not location:
            return response

        redirected_url = requests.compat.urljoin(response.url, location)
        if not self._is_https_upgrade(negotiate_url, redirected_url):
            return response

        parsed = urlparse(redirected_url)
        self.hub_url = (
            f"{parsed.scheme}://{parsed.netloc}"
            f"{parsed.path.removesuffix('/negotiate')}"
        )
        self.log_info(f"SignalR hub_url HTTPS olarak guncellendi: {self.hub_url}")

        return requests.post(
            f"{self.hub_url}/negotiate",
            params={"negotiateVersion": 1},
            timeout=self.http_timeout_s,
            allow_redirects=False,
        )

    @staticmethod
    def _is_https_upgrade(original_url: str, redirected_url: str) -> bool:
        original = urlparse(original_url)
        redirected = urlparse(redirected_url)

        return (
            original.scheme == "http"
            and redirected.scheme == "https"
            and original.netloc == redirected.netloc
            and original.path.rstrip("/") == redirected.path.rstrip("/")
        )

    def _poll_loop(self) -> None:
        handshake_timeout_s = max(self.http_timeout_s, self.poll_timeout_s * 2.0)
        handshake_deadline = time.monotonic() + handshake_timeout_s

        while not self._stop_event.is_set():
            if self._reconnect_requested.is_set():
                raise RuntimeError("SignalR yeniden baglanti istendi")

            try:
                response = requests.get(
                    self.hub_url,
                    params={"id": self._connection_token},
                    timeout=self.poll_timeout_s,
                )
            except requests.exceptions.ReadTimeout:
                if self._reconnect_requested.is_set():
                    raise RuntimeError("SignalR yeniden baglanti istendi")
                if (not self._handshake_ok) and time.monotonic() > handshake_deadline:
                    raise RuntimeError("handshake cevabi gelmedi")
                continue

            if response.status_code == 204:
                raise RuntimeError("poll istegi 204 ile kapandi")

            response.raise_for_status()

            frame_text = response.text or ""
            if not frame_text:
                if (not self._handshake_ok) and time.monotonic() > handshake_deadline:
                    raise RuntimeError("handshake cevabi gelmedi")
                continue

            for raw_message in self._split_frames(frame_text):
                self._handle_signalr_message(raw_message)

    def _send_invocation(self, topic: str, json_text: str) -> None:
        message = {
            "type": 1,
            "target": self.send_method_name,
            "arguments": [topic, json_text],
        }
        self._send_raw_frame(self._encode_frame(message))

    def _send_raw_frame(self, frame_text: str) -> None:
        with self._invoke_lock:
            response = requests.post(
                self.hub_url,
                params={"id": self._connection_token},
                data=frame_text.encode("utf-8"),
                headers={"Content-Type": "text/plain; charset=utf-8"},
                timeout=self.http_timeout_s,
            )

        if response.status_code in (400, 401, 403, 404):
            self._request_reconnect()
            raise RuntimeError(
                f"SignalR connection token gecersiz oldu (HTTP {response.status_code})"
            )

        response.raise_for_status()

    def _request_reconnect(self) -> None:
        self._connected_event.clear()
        self._handshake_ok = False
        self._reconnect_requested.set()

    def _handle_signalr_message(self, raw_message: str) -> None:
        if not raw_message:
            return

        message = json.loads(raw_message)

        if "type" not in message:
            self._handle_handshake_response(message)
            return

        message_type = int(message["type"])

        if message_type == 1:
            self._handle_invocation_message(message)
            return

        if message_type == 6:
            return

        if message_type == 7:
            error_text = str(message.get("error", "SignalR close mesaji geldi"))
            raise RuntimeError(error_text)

    def _handle_handshake_response(self, message: dict) -> None:
        error_text = str(message.get("error", "")).strip()
        if error_text:
            raise RuntimeError(f"handshake hatasi: {error_text}")

        self._handshake_ok = True
        self._connected_event.set()

        if not self._is_connect_log_printed:
            self.log_info(
                f"SignalR baglandi. ConnectionId: {self._connection_id or 'unknown'}"
            )
            self._is_connect_log_printed = True

    def _handle_invocation_message(self, message: dict) -> None:
        target = str(message.get("target", "")).strip()
        arguments = message.get("arguments", [])

        if target != self.receive_method_name:
            return

        if not isinstance(arguments, list) or len(arguments) < 2:
            self.log_warn("SignalR ReceiveMessage argumanlari eksik geldi.")
            return

        topic = str(arguments[0] or "")
        payload = arguments[1]

        if isinstance(payload, str):
            json_text = payload
        else:
            json_text = json.dumps(payload, ensure_ascii=False)

        self.on_receive_message(topic, json_text)

    def _close_connection(self) -> None:
        if not self._connection_token:
            return

        try:
            requests.delete(
                self.hub_url,
                params={"id": self._connection_token},
                timeout=min(self.http_timeout_s, 5.0),
            )
        except Exception:
            pass

        self._connection_id = ""
        self._connection_token = ""

    @classmethod
    def _build_handshake_frame(cls) -> str:
        handshake = {"protocol": "json", "version": 1}
        return cls._encode_frame(handshake)

    @classmethod
    def _encode_frame(cls, message: dict) -> str:
        return (
            json.dumps(message, ensure_ascii=False, separators=(",", ":"))
            + cls.RECORD_SEPARATOR
        )

    @classmethod
    def _split_frames(cls, frame_text: str) -> list[str]:
        parts = frame_text.split(cls.RECORD_SEPARATOR)
        return [part for part in parts if part]
