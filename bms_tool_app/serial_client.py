"""Serial transport for the BMS UART protocol."""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional

from . import protocol

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised manually in GUI.
    serial = None
    list_ports = None


class SerialDependencyError(RuntimeError):
    pass


class BmsTimeoutError(TimeoutError):
    pass


def require_pyserial() -> None:
    if serial is None:
        raise SerialDependencyError(
            "pyserial is not installed. Run: python -m pip install -r requirements.txt"
        )


def list_serial_ports() -> List[str]:
    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


class BmsSerialClient:
    def __init__(
        self,
        baudrate: int = 115200,
        serial_timeout_s: float = 0.05,
        write_timeout_s: float = 0.5,
    ) -> None:
        self.baudrate = baudrate
        self.serial_timeout_s = serial_timeout_s
        self.write_timeout_s = write_timeout_s
        self._serial = None
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    @property
    def port(self) -> Optional[str]:
        if self._serial is None:
            return None
        return self._serial.port

    def open(self, port: str, baudrate: Optional[int] = None) -> None:
        require_pyserial()
        with self._lock:
            self.close()
            if baudrate is not None:
                self.baudrate = baudrate
            self._serial = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout_s,
                write_timeout=self.write_timeout_s,
            )

    def close(self) -> None:
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    def request(
        self,
        command: int,
        payload: bytes = b"",
        timeout_s: float = 1.0,
    ) -> bytes:
        frame = protocol.build_frame(command, payload)
        expected_command = command | protocol.RESPONSE_FLAG
        parser = protocol.FrameParser(max_payload=255)
        deadline = time.monotonic() + timeout_s

        with self._lock:
            if not self.is_open:
                raise RuntimeError("serial port is not open")
            ser = self._serial
            assert ser is not None

            ser.reset_input_buffer()
            ser.write(frame)
            ser.flush()

            while time.monotonic() < deadline:
                waiting = getattr(ser, "in_waiting", 0)
                chunk = ser.read(waiting or 1)
                if not chunk:
                    continue
                for response in parser.feed(chunk):
                    if response.command != expected_command:
                        continue
                    return protocol.decode_response(command, response)

        command_name = protocol.COMMAND_NAMES.get(command, f"0x{command:02X}")
        raise BmsTimeoutError(f"timeout waiting for {command_name} response")

    def ping(self, payload: bytes = b"BMS") -> bytes:
        return self.request(protocol.CMD_PING, payload)

    def read_summary(self) -> Dict[str, object]:
        return protocol.parse_summary(self.request(protocol.CMD_READ_SUMMARY))

    def read_cells(self) -> Dict[str, object]:
        return protocol.parse_cells(self.request(protocol.CMD_READ_CELLS))

    def read_faults(self) -> Dict[str, object]:
        return protocol.parse_faults(self.request(protocol.CMD_READ_FAULTS))

    def read_limits(self) -> Dict[str, object]:
        return protocol.parse_limits(self.request(protocol.CMD_READ_LIMITS))

    def otp_check(self) -> Dict[str, object]:
        return protocol.parse_otp_status(self.request(protocol.CMD_OTP_CHECK, timeout_s=2.0))

    def otp_read(self) -> Dict[str, object]:
        return protocol.parse_otp_status(self.request(protocol.CMD_OTP_READ, timeout_s=2.0))

    def otp_write(self) -> Dict[str, object]:
        data = self.request(
            protocol.CMD_OTP_WRITE,
            protocol.OTP_WRITE_MAGIC,
            timeout_s=5.0,
        )
        return protocol.parse_otp_status(data)

    def calibrate_current(self, actual_ma: int) -> Dict[str, object]:
        data = self.request(
            protocol.CMD_CALIBRATE_CURRENT,
            protocol.make_calibration_payload(actual_ma),
            timeout_s=3.0,
        )
        return protocol.parse_current_calibration_result(data)

    def read_all(self) -> Dict[str, object]:
        result: Dict[str, object] = {"errors": {}}
        for key, reader in (
            ("summary", self.read_summary),
            ("cells", self.read_cells),
            ("faults", self.read_faults),
            ("limits", self.read_limits),
        ):
            try:
                result[key] = reader()
            except Exception as exc:  # Keep the UI useful if one command fails.
                result["errors"][key] = str(exc)
        return result
