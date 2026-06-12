"""Serial transport for the BMS UART protocol."""

from __future__ import annotations

import logging
import threading
import time
from typing import Dict, List, Optional

from . import protocol


logger = logging.getLogger(__name__)

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
        logger.warning("Cannot list serial ports because pyserial is not installed")
        return []
    ports = [port.device for port in list_ports.comports()]
    logger.info("Detected serial ports: %s", ", ".join(ports) if ports else "none")
    return ports


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
            logger.info(
                "Opening serial port %s at %s baud, timeout=%.3fs, write_timeout=%.3fs",
                port,
                self.baudrate,
                self.serial_timeout_s,
                self.write_timeout_s,
            )
            self._serial = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout_s,
                write_timeout=self.write_timeout_s,
            )
            logger.info("Serial port %s opened", port)

    def close(self) -> None:
        with self._lock:
            if self._serial is not None:
                port = self._serial.port
                logger.info("Closing serial port %s", port)
                try:
                    self._serial.close()
                finally:
                    self._serial = None
                    logger.info("Serial port %s closed", port)

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
        started_at = time.monotonic()
        command_name = protocol.COMMAND_NAMES.get(command, f"0x{command:02X}")
        logger.info(
            "Request %s started: payload_len=%d timeout=%.2fs",
            command_name,
            len(payload),
            timeout_s,
        )
        logger.debug("TX %s: %s", command_name, protocol.to_hex(frame))

        with self._lock:
            if not self.is_open:
                logger.error("Request %s failed: serial port is not open", command_name)
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
                logger.debug("RX chunk for %s: %s", command_name, protocol.to_hex(chunk))
                for response in parser.feed(chunk):
                    logger.debug(
                        "RX frame for %s: command=0x%02X payload_len=%d payload=%s",
                        command_name,
                        response.command,
                        len(response.payload),
                        protocol.to_hex(response.payload),
                    )
                    if response.command != expected_command:
                        logger.warning(
                            "Ignoring response command 0x%02X while waiting for 0x%02X",
                            response.command,
                            expected_command,
                        )
                        continue
                    try:
                        data = protocol.decode_response(command, response)
                    except protocol.BmsStatusError as exc:
                        logger.warning(
                            "Request %s returned status %s with data_len=%d",
                            command_name,
                            protocol.STATUS_NAMES.get(exc.status, f"0x{exc.status:02X}"),
                            len(exc.data),
                        )
                        raise
                    except Exception:
                        logger.exception("Request %s response decode failed", command_name)
                        raise
                    elapsed_ms = (time.monotonic() - started_at) * 1000
                    logger.info(
                        "Request %s completed in %.0f ms: response_len=%d",
                        command_name,
                        elapsed_ms,
                        len(data),
                    )
                    return data

        elapsed_ms = (time.monotonic() - started_at) * 1000
        logger.warning("Request %s timed out after %.0f ms", command_name, elapsed_ms)
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
                logger.info("read_all %s succeeded", key)
            except Exception as exc:  # Keep the UI useful if one command fails.
                result["errors"][key] = str(exc)
                logger.warning("read_all %s failed: %s", key, exc)
        return result
