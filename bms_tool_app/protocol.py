"""BMS UART protocol helpers.

The protocol is derived from reference/bms_uart.h and reference/bms_uart.c.
Frame format:
    AA 55 CMD LEN PAYLOAD CRC_LO CRC_HI

CRC is CRC16/Modbus over CMD, LEN and PAYLOAD.
Responses use CMD | 0x80 and have a status byte as the first payload byte.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Dict, Iterable, List, Optional


SOF0 = 0xAA
SOF1 = 0x55
RESPONSE_FLAG = 0x80
PROTOCOL_VERSION = 0x04
MAX_PAYLOAD_SIZE = 160
OTP_WRITE_MAGIC = b"OTP!"
RAW_TRACKING_SIZE = 124
RAW_TRACKING_SHORT_ENUM_SIZE = 152
RAW_TRACKING_STANDARD_ENUM_SIZE = 160
COMPACT_SUMMARY_SIZE = 51
COMPACT_SUMMARY_WITH_DISCHARGE_SIZE = 55

CMD_PING = 0x01
CMD_READ_SUMMARY = 0x10
CMD_READ_CELLS = 0x11
CMD_READ_FAULTS = 0x12
CMD_READ_LIMITS = 0x13
CMD_READ_CURRENT = 0x14
CMD_READ_STATUS = 0x15
CMD_READ_PACK_VOLTAGE = 0x16
CMD_READ_CELL_COUNT = 0x17
CMD_READ_TEMPERATURE = 0x18
CMD_READ_CYCLE_COUNT = 0x19
CMD_OTP_CHECK = 0x20
CMD_OTP_WRITE = 0x21
CMD_OTP_READ = 0x22
CMD_CALIBRATE_CURRENT = 0x30
CMD_SET_BALANCING = 0x31
CMD_PROTECTION_EVENT = 0x40

COMMAND_NAMES = {
    CMD_PING: "PING",
    CMD_READ_SUMMARY: "READ_SUMMARY",
    CMD_READ_CELLS: "READ_CELLS",
    CMD_READ_FAULTS: "READ_FAULTS",
    CMD_READ_LIMITS: "READ_LIMITS",
    CMD_READ_CURRENT: "READ_CURRENT",
    CMD_READ_STATUS: "READ_STATUS",
    CMD_READ_PACK_VOLTAGE: "READ_PACK_VOLTAGE",
    CMD_READ_CELL_COUNT: "READ_CELL_COUNT",
    CMD_READ_TEMPERATURE: "READ_TEMPERATURE",
    CMD_READ_CYCLE_COUNT: "READ_CYCLE_COUNT",
    CMD_OTP_CHECK: "OTP_CHECK",
    CMD_OTP_WRITE: "OTP_WRITE",
    CMD_OTP_READ: "OTP_READ",
    CMD_CALIBRATE_CURRENT: "CALIBRATE_CURRENT",
    CMD_SET_BALANCING: "SET_BALANCING",
    CMD_PROTECTION_EVENT: "PROTECTION_EVENT",
}

STATUS_OK = 0x00
STATUS_BAD_LENGTH = 0x01
STATUS_BAD_COMMAND = 0x02
STATUS_BUSY = 0x03
STATUS_INTERNAL_ERROR = 0x04
STATUS_BAD_PAYLOAD = 0x05

STATUS_NAMES = {
    STATUS_OK: "OK",
    STATUS_BAD_LENGTH: "BAD_LENGTH",
    STATUS_BAD_COMMAND: "BAD_COMMAND",
    STATUS_BUSY: "BUSY",
    STATUS_INTERNAL_ERROR: "INTERNAL_ERROR",
    STATUS_BAD_PAYLOAD: "BAD_PAYLOAD",
}

STATE_NAMES = {
    0: "INIT",
    1: "NORMAL",
    2: "CHARGE_PROTECT",
    3: "DISCHARGE_PROTECT",
    4: "FAULT",
}

CURRENT_DIRECTION_NAMES = {
    0: "IDLE",
    1: "CHARGE",
    2: "DISCHARGE",
}

FAULT_NAMES = [
    "Cell over voltage",
    "Cell under voltage",
    "Charge over temperature",
    "Discharge over temperature",
    "Under temperature",
    "Charge over current",
    "Discharge over current",
    "Short circuit",
    "BQ safety fault",
    "Communication fault",
]

PROTECTION_REASON_NAMES = {
    0x01: "Cell over voltage",
    0x02: "Cell under voltage",
    0x03: "Charge over temperature",
    0x04: "Discharge over temperature",
    0x05: "Under temperature",
    0x06: "Charge over current",
    0x07: "Discharge over current",
    0x08: "Short circuit",
    0x09: "Communication fault",
}

PROTECTION_REASON_FAULT_NAMES = {
    0x01: "Cell over voltage",
    0x02: "Cell under voltage",
    0x03: "Charge over temperature",
    0x04: "Discharge over temperature",
    0x05: "Under temperature",
    0x06: "Charge over current",
    0x07: "Discharge over current",
    0x08: "Short circuit",
    0x09: "Communication fault",
}

GATE_SIGNAL_NAMES = [
    "DCHG pin active",
    "DDSG pin active",
]

FET_NAMES = [
    "FETs enabled",
    "Charge FET enabled",
    "Discharge FET enabled",
    "Charge disabled",
    "Discharge disabled",
    "FETOFF asserted",
]

OTP_FLAG_NAMES = [
    "Full access OK",
    "Config update OK",
    "OTP check OK",
    "OTP write OK",
    "OTP blocked",
    "OTP pending",
    "DCHG active high",
    "DDSG active high",
    "DA user volts",
    "OTP result lock",
    "OTP result no signature",
    "OTP result no data",
    "OTP result high temperature",
    "OTP result low voltage",
    "OTP result high voltage",
    "OTP write failed",
]

CALIBRATION_STATUS_NAMES = {
    0: "OK",
    1: "ACTUAL_CURRENT_ZERO",
    2: "MEASURED_CURRENT_ZERO",
    3: "DEVIATION_TOO_HIGH",
    4: "WRITE_FAILED",
}

STATUS_STATE_FLAG_NAMES = [
    "Charging",
    "Discharging",
    "Charge FET enabled",
    "Discharge FET enabled",
    "Balancing active",
    "Balancing enabled",
]

STATUS_FAULT_FLAG_NAMES = [
    "Cell over voltage",
    "Cell under voltage",
    "Charge over current",
    "Discharge over current",
    "Charge/discharge over temperature",
    "Under temperature",
    "Short circuit",
]


class ProtocolError(Exception):
    """Base protocol exception."""


class BmsStatusError(ProtocolError):
    """Raised when the device returns a non-OK status."""

    def __init__(self, command: int, status: int, data: bytes = b"") -> None:
        self.command = command
        self.status = status
        self.data = data
        command_name = COMMAND_NAMES.get(command, f"0x{command:02X}")
        status_name = STATUS_NAMES.get(status, f"0x{status:02X}")
        super().__init__(f"{command_name} returned {status_name}")


class PayloadTooLargeError(ProtocolError):
    """Raised when a request payload cannot fit in one protocol frame."""


@dataclass(frozen=True)
class Frame:
    command: int
    payload: bytes


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def build_frame(command: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF:
        raise ValueError("command must fit in one byte")
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise PayloadTooLargeError(
            f"payload length {len(payload)} exceeds firmware limit {MAX_PAYLOAD_SIZE}"
        )
    header = bytes([SOF0, SOF1, command, len(payload)])
    crc = crc16_modbus(bytes([command, len(payload)]) + payload)
    return header + payload + struct.pack("<H", crc)


class FrameParser:
    """Streaming parser for BMS UART frames."""

    def __init__(self, max_payload: int = MAX_PAYLOAD_SIZE) -> None:
        self.max_payload = max_payload
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []

        while True:
            if len(self._buffer) < 2:
                return frames

            try:
                sof_index = self._buffer.index(SOF0)
            except ValueError:
                self._buffer.clear()
                return frames

            if sof_index:
                del self._buffer[:sof_index]

            if len(self._buffer) < 4:
                return frames

            if self._buffer[1] != SOF1:
                del self._buffer[0]
                continue

            length = self._buffer[3]
            if length > self.max_payload:
                del self._buffer[0]
                continue

            frame_length = 6 + length
            if len(self._buffer) < frame_length:
                return frames

            raw = bytes(self._buffer[:frame_length])
            del self._buffer[:frame_length]

            command = raw[2]
            payload = raw[4 : 4 + length]
            received_crc = struct.unpack_from("<H", raw, 4 + length)[0]
            calculated_crc = crc16_modbus(raw[2 : 4 + length])
            if received_crc == calculated_crc:
                frames.append(Frame(command, payload))

        return frames


def decode_response(request_command: int, response: Frame) -> bytes:
    expected_command = request_command | RESPONSE_FLAG
    if response.command != expected_command:
        got = f"0x{response.command:02X}"
        expected = f"0x{expected_command:02X}"
        raise ProtocolError(f"unexpected response command {got}, expected {expected}")
    if not response.payload:
        raise ProtocolError("response payload is missing the status byte")

    status = response.payload[0]
    data = response.payload[1:]
    if status != STATUS_OK:
        raise BmsStatusError(request_command, status, data)
    return data


def make_calibration_payload(actual_ma: int) -> bytes:
    return struct.pack("<i", int(actual_ma))


def make_balancing_payload(enabled: object) -> bytes:
    mode = int(enabled)
    if mode not in (0, 1):
        raise ValueError("balancing mode must be 0 (disabled) or 1 (enabled)")
    return bytes([mode])


def active_flag_names(bitmap: int, names: Iterable[str]) -> List[str]:
    return [name for bit, name in enumerate(names) if bitmap & (1 << bit)]


class PayloadReader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0

    def remaining(self) -> int:
        return len(self.data) - self.pos

    def require(self, size: int) -> None:
        if self.remaining() < size:
            raise ProtocolError(
                f"payload too short at offset {self.pos}; need {size}, have {self.remaining()}"
            )

    def u8(self) -> int:
        self.require(1)
        value = self.data[self.pos]
        self.pos += 1
        return value

    def bool(self) -> bool:
        return self.u8() != 0

    def u16(self) -> int:
        self.require(2)
        value = struct.unpack_from("<H", self.data, self.pos)[0]
        self.pos += 2
        return value

    def i16(self) -> int:
        self.require(2)
        value = struct.unpack_from("<h", self.data, self.pos)[0]
        self.pos += 2
        return value

    def u32(self) -> int:
        self.require(4)
        value = struct.unpack_from("<I", self.data, self.pos)[0]
        self.pos += 4
        return value

    def i32(self) -> int:
        self.require(4)
        value = struct.unpack_from("<i", self.data, self.pos)[0]
        self.pos += 4
        return value

    def u64(self) -> int:
        self.require(8)
        value = struct.unpack_from("<Q", self.data, self.pos)[0]
        self.pos += 8
        return value

    def finish(self) -> None:
        if self.remaining() != 0:
            raise ProtocolError(
                f"payload has {self.remaining()} unexpected byte(s) at offset {self.pos}"
            )


def _require_length(data: bytes, expected: int, name: str) -> None:
    if len(data) != expected:
        raise ProtocolError(
            f"{name} payload must be {expected} byte(s); got {len(data)}"
        )


def _u8(data: bytes, offset: int) -> int:
    return data[offset]


def _bool(data: bytes, offset: int) -> bool:
    return _u8(data, offset) != 0


def _u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<i", data, offset)[0]


def _u64(data: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", data, offset)[0]


def _align(offset: int, alignment: int) -> int:
    return (offset + alignment - 1) & ~(alignment - 1)


def _fault_bitmap_from_bools(values: Iterable[bool]) -> int:
    bitmap = 0
    for bit, active in enumerate(values):
        if active:
            bitmap |= 1 << bit
    return bitmap


def _fet_bitmap_from_summary(summary: Dict[str, object]) -> int:
    bitmap = 0
    bitmap |= (1 << 0) if summary.get("fets_enabled") else 0
    bitmap |= (1 << 1) if summary.get("charge_fet_enabled") else 0
    bitmap |= (1 << 2) if summary.get("discharge_fet_enabled") else 0
    bitmap |= (1 << 3) if summary.get("charge_disabled") else 0
    bitmap |= (1 << 4) if summary.get("discharge_disabled") else 0
    bitmap |= (1 << 5) if summary.get("fetoff_asserted") else 0
    return bitmap


def parse_cells(data: bytes) -> Dict[str, object]:
    reader = PayloadReader(data)
    count = reader.u8()
    voltages = [reader.u16() for _ in range(count)]
    reader.finish()
    return {
        "cell_count": count,
        "cell_voltages_mV": voltages,
        "min_cell_voltage_mV": min(voltages) if voltages else None,
        "max_cell_voltage_mV": max(voltages) if voltages else None,
        "average_cell_voltage_mV": round(sum(voltages) / len(voltages), 1)
        if voltages
        else None,
        "delta_cell_voltage_mV": (max(voltages) - min(voltages)) if voltages else None,
    }


def parse_faults(data: bytes) -> Dict[str, object]:
    _require_length(data, 8, "READ_FAULTS")
    reader = PayloadReader(data)
    fault_bitmap = reader.u16()
    gate_signal_bitmap = reader.u8()
    alert_active = reader.bool()
    alert_counter = reader.u32()
    return {
        "fault_bitmap": fault_bitmap,
        "faults": active_flag_names(fault_bitmap, FAULT_NAMES),
        "gate_signal_bitmap": gate_signal_bitmap,
        "gate_signals": active_flag_names(gate_signal_bitmap, GATE_SIGNAL_NAMES),
        "alert_active": alert_active,
        "alert_counter": alert_counter,
    }


def parse_protection_event(data: bytes) -> Dict[str, object]:
    _require_length(data, 1, "PROTECTION_EVENT")
    reader = PayloadReader(data)
    reason = reader.u8()
    return {
        "reason": reason,
        "reason_name": PROTECTION_REASON_NAMES.get(reason, f"UNKNOWN(0x{reason:02X})"),
        "fault_name": PROTECTION_REASON_FAULT_NAMES.get(reason),
        "raw_hex": to_hex(data),
    }


def parse_limits(data: bytes) -> Dict[str, object]:
    _require_length(data, 32, "READ_LIMITS")
    reader = PayloadReader(data)
    return {
        "cell_count": reader.u8(),
        "thermistor_count": reader.u8(),
        "cell_ov_cutoff_mV": reader.u16(),
        "cell_ov_recover_mV": reader.u16(),
        "cell_uv_cutoff_mV": reader.u16(),
        "cell_uv_recover_mV": reader.u16(),
        "balance_delta_mV": reader.u16(),
        "balance_min_cell_mV": reader.u16(),
        "over_current_mA": reader.i32(),
        "short_circuit_mA": reader.i32(),
        "charge_ot_cutoff_C": reader.i16(),
        "discharge_ot_cutoff_C": reader.i16(),
        "undertemp_cutoff_C": reader.i16(),
        "nominal_capacity_mAh": reader.u32(),
    }


def parse_current(data: bytes) -> Dict[str, object]:
    _require_length(data, 4, "READ_CURRENT")
    return {"current_mA": struct.unpack("<i", data)[0]}


def parse_status(data: bytes) -> Dict[str, object]:
    _require_length(data, 4, "READ_STATUS")
    soc_percent, state_flags, fault_flags = struct.unpack("<BBH", data)
    return {
        "soc_percent": soc_percent,
        "state_flags": state_flags,
        "state_flag_names": active_flag_names(state_flags, STATUS_STATE_FLAG_NAMES),
        "fault_flags": fault_flags,
        "fault_flag_names": active_flag_names(fault_flags, STATUS_FAULT_FLAG_NAMES),
        "charging": bool(state_flags & (1 << 0)),
        "discharging": bool(state_flags & (1 << 1)),
        "charge_fet_enabled": bool(state_flags & (1 << 2)),
        "discharge_fet_enabled": bool(state_flags & (1 << 3)),
        "balancing_active": bool(state_flags & (1 << 4)),
        "balancing_enabled": bool(state_flags & (1 << 5)),
    }


def parse_pack_voltage(data: bytes) -> Dict[str, object]:
    _require_length(data, 4, "READ_PACK_VOLTAGE")
    return {"pack_voltage_mV": struct.unpack("<I", data)[0]}


def parse_cell_count(data: bytes) -> Dict[str, object]:
    _require_length(data, 1, "READ_CELL_COUNT")
    return {"cell_count": data[0]}


def parse_temperature(data: bytes) -> Dict[str, object]:
    _require_length(data, 2, "READ_TEMPERATURE")
    temperature_deci_c = struct.unpack("<h", data)[0]
    return {
        "temperature_deciC": temperature_deci_c,
        "temperature_C": temperature_deci_c / 10.0,
    }


def parse_cycle_count(data: bytes) -> Dict[str, object]:
    _require_length(data, 4, "READ_CYCLE_COUNT")
    return {"completed_cycle_count": struct.unpack("<I", data)[0]}


def parse_balancing(data: bytes) -> Dict[str, object]:
    _require_length(data, 1, "SET_BALANCING")
    mode = data[0]
    if mode not in (0, 1):
        raise ProtocolError(f"SET_BALANCING returned invalid mode 0x{mode:02X}")
    return {"mode": mode, "enabled": bool(mode)}


def parse_otp_status(data: bytes) -> Dict[str, object]:
    _require_length(data, 27, "OTP status")
    reader = PayloadReader(data)
    flags = reader.u16()
    result = {
        "flags": flags,
        "flag_names": active_flag_names(flags, OTP_FLAG_NAMES),
        "security_state": reader.u8(),
        "check_result": reader.u8(),
        "check_data_fail_addr": reader.u16(),
        "write_result": reader.u8(),
        "write_data_fail_addr": reader.u16(),
        "battery_status_raw": reader.u16(),
        "static_config_signature": reader.u16(),
        "stack_voltage_mV": reader.u16(),
        "pack_voltage_mV": reader.u16(),
        "internal_temp_C": reader.i16(),
        "reg0_config": reader.u8(),
        "reg12_control": reader.u8(),
        "da_config": reader.u8(),
        "vcell_mode": reader.u16(),
        "dchg_pin_config": reader.u8(),
        "ddsg_pin_config": reader.u8(),
        "dfetoff_pin_config": reader.u8(),
    }
    return result


def parse_current_calibration_result(
    data: bytes,
    status: Optional[int] = None,
    actual_ma: Optional[int] = None,
) -> Dict[str, object]:
    if status is None and len(data) >= 24:
        status, actual, measured, deviation, old_gain, new_gain = struct.unpack_from(
            "<IiiIII", data, 0
        )
        return {
            "status": status,
            "status_name": CALIBRATION_STATUS_NAMES.get(status, f"0x{status:X}"),
            "actual_mA": actual,
            "measured_mA": measured,
            "deviation_ppm": deviation,
            "old_gain_ppm": old_gain,
            "new_gain_ppm": new_gain,
        }
    if status is None and len(data) >= 21:
        status = data[0]
        actual, measured, deviation, old_gain, new_gain = struct.unpack_from(
            "<iiIII", data, 1
        )
        return {
            "status": status,
            "status_name": CALIBRATION_STATUS_NAMES.get(status, f"0x{status:X}"),
            "actual_mA": actual,
            "measured_mA": measured,
            "deviation_ppm": deviation,
            "old_gain_ppm": old_gain,
            "new_gain_ppm": new_gain,
        }
    if len(data) < 12:
        raise ProtocolError(
            f"calibration result payload too short: {len(data)} bytes; need 12"
        )
    measured, deviation, new_gain = struct.unpack_from("<iII", data, 0)
    status = STATUS_OK if status is None else status
    result = {
        "status": status,
        "status_name": CALIBRATION_STATUS_NAMES.get(status, f"0x{status:X}"),
        "measured_mA": measured,
        "deviation_ppm": deviation,
        "new_gain_ppm": new_gain,
    }
    if actual_ma is not None:
        result["actual_mA"] = actual_ma
    return result


def parse_summary(data: bytes) -> Dict[str, object]:
    """Parse the current 124-byte raw tracking payload or a legacy variant.

    Protocol document v1.3 defines the current GCC/ARM layout as 124 bytes.
    Older compact and 152/160-byte raw layouts remain readable so saved captures
    from older firmware do not become unusable.
    """

    if len(data) == RAW_TRACKING_SIZE:
        return parse_current_tracking_summary(data)
    if len(data) >= COMPACT_SUMMARY_SIZE and data[0] == PROTOCOL_VERSION:
        return parse_compact_summary(data)
    if len(data) >= RAW_TRACKING_SHORT_ENUM_SIZE:
        return parse_tracking_summary(data)
    return {
        "summary_format": "unknown",
        "payload_length": len(data),
        "raw_hex": data.hex(" "),
    }


def parse_current_tracking_summary(data: bytes) -> Dict[str, object]:
    """Parse the 124-byte BMS_Tracking_t layout documented by protocol v1.3."""

    _require_length(data, RAW_TRACKING_SIZE, "READ_SUMMARY")
    initialized = _bool(data, 0)
    connected = _bool(data, 1)
    state = _u8(data, 2)
    current_direction = _u8(data, 3)
    cell_index_accumulated = list(data[4:14])
    real_time_accumulated = [_u16(data, 14 + i * 2) for i in range(10)]
    cell_voltages = [_u16(data, 34 + i * 2) for i in range(10)]
    fault_values = [_bool(data, 82 + i) for i in range(len(FAULT_NAMES))]
    fault_bitmap = _fault_bitmap_from_bools(fault_values)
    charge_gate_signal = _bool(data, 94)
    discharge_gate_signal = _bool(data, 95)
    gate_signal_bitmap = (
        (1 if charge_gate_signal else 0)
        | (2 if discharge_gate_signal else 0)
    )

    summary: Dict[str, object] = {
        "summary_format": "raw_tracking_v1_3",
        "payload_length": len(data),
        "parsed_length": RAW_TRACKING_SIZE,
        "initialized": initialized,
        "connected": connected,
        "state": state,
        "state_name": STATE_NAMES.get(state, f"UNKNOWN({state})"),
        "current_direction": current_direction,
        "current_direction_name": CURRENT_DIRECTION_NAMES.get(
            current_direction, f"UNKNOWN({current_direction})"
        ),
        "cell_index_accumulated": cell_index_accumulated,
        "cell_real_time_accumulated": real_time_accumulated,
        "cell_voltages_mV": cell_voltages,
        "min_cell_voltage_mV": _u16(data, 54),
        "max_cell_voltage_mV": _u16(data, 56),
        "average_cell_voltage_mV": _u16(data, 58),
        "delta_cell_voltage_mV": _u16(data, 60),
        "pack_voltage_mV": _u16(data, 62),
        "current_mA": _i32(data, 64),
        "temperature_C": [_i16(data, 68), _i16(data, 70)],
        "charging": _bool(data, 72),
        "discharging": _bool(data, 73),
        "charge_fet_enabled": _bool(data, 74),
        "discharge_fet_enabled": _bool(data, 75),
        "fets_enabled": _bool(data, 76),
        "bq_charge_fet_blocked": _bool(data, 77),
        "bq_discharge_fet_blocked": _bool(data, 78),
        "bq_alarm_raw_status": _u16(data, 80),
        "fault_bitmap": fault_bitmap,
        "faults": active_flag_names(fault_bitmap, FAULT_NAMES),
        "charge_disabled": _bool(data, 92),
        "discharge_disabled": _bool(data, 93),
        "charge_gate_fault_signal": charge_gate_signal,
        "discharge_gate_fault_signal": discharge_gate_signal,
        "gate_signal_bitmap": gate_signal_bitmap,
        "gate_signals": active_flag_names(gate_signal_bitmap, GATE_SIGNAL_NAMES),
        "fetoff_asserted": _bool(data, 96),
        "alert_active": _bool(data, 97),
        "alert_counter": _u32(data, 100),
        "bq_sleep_mode": _bool(data, 104),
        "bq_sleep_allowed": _bool(data, 105),
        "bat_sense_enabled": _bool(data, 106),
        "bat_adc_estimated_pack_mV": _u16(data, 108),
        "balance_required": _bool(data, 110),
        "balance_mask": _u16(data, 112),
        "completed_cycle_count": _u32(data, 116),
        "current_calibration_gain_ppm": _u32(data, 120),
    }
    fet_bitmap = _fet_bitmap_from_summary(summary)
    summary["fet_bitmap"] = fet_bitmap
    summary["fets"] = active_flag_names(fet_bitmap, FET_NAMES)
    return summary


def parse_compact_summary(data: bytes) -> Dict[str, object]:
    reader = PayloadReader(data)
    version = reader.u8()
    uptime_ms = reader.u32()
    initialized = reader.bool()
    connected = reader.bool()
    state = reader.u8()
    current_direction = reader.u8()
    fault_bitmap = reader.u16()
    stack_voltage = reader.u16()
    pack_voltage = reader.u16()
    bat_adc_pack = reader.u16()
    current = reader.i32()
    min_cell = reader.u16()
    max_cell = reader.u16()
    avg_cell = reader.u16()
    delta_cell = reader.u16()
    temps = [reader.i16(), reader.i16()]
    charge_throughput = reader.u32()
    discharge_throughput = None
    if len(data) >= COMPACT_SUMMARY_WITH_DISCHARGE_SIZE:
        discharge_throughput = reader.u32()
    equivalent_cycles = reader.u32()
    fet_bitmap = reader.u8()
    balance_required = reader.bool()
    balance_mask = reader.u16()
    alert_counter = reader.u32()
    circle_counter = reader.u16()

    result = {
        "summary_format": "compact",
        "protocol_version": version,
        "uptime_ms": uptime_ms,
        "initialized": initialized,
        "connected": connected,
        "state": state,
        "state_name": STATE_NAMES.get(state, f"UNKNOWN({state})"),
        "current_direction": current_direction,
        "current_direction_name": CURRENT_DIRECTION_NAMES.get(
            current_direction, f"UNKNOWN({current_direction})"
        ),
        "fault_bitmap": fault_bitmap,
        "faults": active_flag_names(fault_bitmap, FAULT_NAMES),
        "stack_voltage_mV": stack_voltage,
        "pack_voltage_mV": pack_voltage,
        "bat_adc_estimated_pack_mV": bat_adc_pack,
        "current_mA": current,
        "min_cell_voltage_mV": min_cell,
        "max_cell_voltage_mV": max_cell,
        "average_cell_voltage_mV": avg_cell,
        "delta_cell_voltage_mV": delta_cell,
        "temperature_C": temps,
        "charge_throughput_mAh": charge_throughput,
        "equivalent_cycle_milliCycles": equivalent_cycles,
        "fet_bitmap": fet_bitmap,
        "fets": active_flag_names(fet_bitmap, FET_NAMES),
        "balance_required": balance_required,
        "balance_mask": balance_mask,
        "alert_counter": alert_counter,
        "circle_counter": circle_counter,
    }
    if discharge_throughput is not None:
        result["discharge_throughput_mAh"] = discharge_throughput
    return result


def parse_tracking_summary(data: bytes) -> Dict[str, object]:
    """Parse BMS_Tracking_t as emitted by the current C code.

    The firmware may be built with normal 4-byte enums, which makes the raw
    struct 160 bytes, or with short 1-byte enums, which makes it 152 bytes.
    """

    if len(data) >= RAW_TRACKING_STANDARD_ENUM_SIZE:
        enum_size = 4
        enum_reader = _u32
        summary_format = "raw_tracking"
    elif len(data) >= RAW_TRACKING_SHORT_ENUM_SIZE:
        enum_size = 1
        enum_reader = _u8
        summary_format = "raw_tracking_short_enums"
    else:
        raise ProtocolError(
            "raw tracking payload needs at least "
            f"{RAW_TRACKING_SHORT_ENUM_SIZE} bytes; got {len(data)}"
        )

    offset = 0
    initialized = _bool(data, offset)
    offset += 1
    connected = _bool(data, offset)
    offset += 1

    offset = _align(offset, enum_size)
    state = enum_reader(data, offset)
    offset += enum_size
    current_direction = enum_reader(data, offset)
    offset += enum_size

    offset = _align(offset, 2)
    cell_index_accumulated = list(data[offset : offset + 10])
    offset += 10
    offset = _align(offset, 2)
    real_time_accumulated = [_u16(data, offset + i * 2) for i in range(10)]
    offset += 20
    cell_voltages = [_u16(data, offset + i * 2) for i in range(10)]
    offset += 20
    min_cell = _u16(data, offset)
    offset += 2
    max_cell = _u16(data, offset)
    offset += 2
    avg_cell = _u16(data, offset)
    offset += 2
    delta_cell = _u16(data, offset)
    offset += 2

    stack_voltage = _u16(data, offset)
    offset += 2
    pack_voltage = _u16(data, offset)
    offset += 2
    circle_counter = _u16(data, offset)
    offset += 2

    offset = _align(offset, 4)
    current_ma = _i32(data, offset)
    offset += 4
    temperatures = [_i16(data, offset), _i16(data, offset + 2)]
    offset += 4

    charging = _bool(data, offset)
    offset += 1
    discharging = _bool(data, offset)
    offset += 1
    charge_fet_enabled = _bool(data, offset)
    offset += 1
    discharge_fet_enabled = _bool(data, offset)
    offset += 1
    fets_enabled = _bool(data, offset)
    offset += 1
    bq_charge_fet_blocked = _bool(data, offset)
    offset += 1
    bq_discharge_fet_blocked = _bool(data, offset)
    offset += 1

    offset = _align(offset, 2)
    bq_alarm_raw_status = _u16(data, offset)
    offset += 2

    fault_values = [_bool(data, offset + i) for i in range(len(FAULT_NAMES))]
    offset += len(FAULT_NAMES)
    fault_bitmap = _fault_bitmap_from_bools(fault_values)

    charge_disabled = _bool(data, offset)
    offset += 1
    discharge_disabled = _bool(data, offset)
    offset += 1
    charge_gate_fault_signal = _bool(data, offset)
    offset += 1
    discharge_gate_fault_signal = _bool(data, offset)
    offset += 1
    fetoff_asserted = _bool(data, offset)
    offset += 1
    alert_active = _bool(data, offset)
    offset += 1
    gate_signal_bitmap = 0
    gate_signal_bitmap |= 1 << 0 if charge_gate_fault_signal else 0
    gate_signal_bitmap |= 1 << 1 if discharge_gate_fault_signal else 0

    offset = _align(offset, 4)
    alert_counter = _u32(data, offset)
    offset += 4
    bq_sleep_mode = _bool(data, offset)
    offset += 1
    bq_sleep_allowed = _bool(data, offset)
    offset += 1
    bat_sense_enabled = _bool(data, offset)
    offset += 1

    offset = _align(offset, 2)
    bat_adc_pack = _u16(data, offset)
    offset += 2
    balance_required = _bool(data, offset)
    offset += 1
    offset = _align(offset, 2)
    balance_mask = _u16(data, offset)
    offset += 2

    offset = _align(offset, 8)
    charge_accumulated = _u64(data, offset)
    offset += 8
    discharge_accumulated = _u64(data, offset)
    offset += 8
    charge_throughput = _u32(data, offset)
    offset += 4
    equivalent_cycles = _u32(data, offset)
    offset += 4
    current_calibration_gain = _u32(data, offset)
    offset += 4

    summary: Dict[str, object] = {
        "summary_format": summary_format,
        "payload_length": len(data),
        "parsed_length": offset,
        "initialized": initialized,
        "connected": connected,
        "state": state,
        "current_direction": current_direction,
        "cell_index_accumulated": cell_index_accumulated,
        "cell_real_time_accumulated": real_time_accumulated,
        "cell_voltages_mV": cell_voltages,
        "min_cell_voltage_mV": min_cell,
        "max_cell_voltage_mV": max_cell,
        "average_cell_voltage_mV": avg_cell,
        "delta_cell_voltage_mV": delta_cell,
        "stack_voltage_mV": stack_voltage,
        "pack_voltage_mV": pack_voltage,
        "circle_counter": circle_counter,
        "current_mA": current_ma,
        "temperature_C": temperatures,
        "charging": charging,
        "discharging": discharging,
        "charge_fet_enabled": charge_fet_enabled,
        "discharge_fet_enabled": discharge_fet_enabled,
        "fets_enabled": fets_enabled,
        "bq_charge_fet_blocked": bq_charge_fet_blocked,
        "bq_discharge_fet_blocked": bq_discharge_fet_blocked,
        "bq_alarm_raw_status": bq_alarm_raw_status,
        "fault_bitmap": fault_bitmap,
        "faults": active_flag_names(fault_bitmap, FAULT_NAMES),
        "charge_disabled": charge_disabled,
        "discharge_disabled": discharge_disabled,
        "charge_gate_fault_signal": charge_gate_fault_signal,
        "discharge_gate_fault_signal": discharge_gate_fault_signal,
        "gate_signal_bitmap": gate_signal_bitmap,
        "gate_signals": active_flag_names(gate_signal_bitmap, GATE_SIGNAL_NAMES),
        "fetoff_asserted": fetoff_asserted,
        "alert_active": alert_active,
        "alert_counter": alert_counter,
        "bq_sleep_mode": bq_sleep_mode,
        "bq_sleep_allowed": bq_sleep_allowed,
        "bat_sense_enabled": bat_sense_enabled,
        "bat_adc_estimated_pack_mV": bat_adc_pack,
        "balance_required": balance_required,
        "balance_mask": balance_mask,
        "charge_accumulated_mAs": charge_accumulated,
        "discharge_accumulated_mAs": discharge_accumulated,
        "charge_throughput_mAh": charge_throughput,
        "equivalent_cycle_milliCycles": equivalent_cycles,
        "current_calibration_gain_ppm": current_calibration_gain,
    }
    state = int(summary["state"])
    direction = int(summary["current_direction"])
    summary["state_name"] = STATE_NAMES.get(state, f"UNKNOWN({state})")
    summary["current_direction_name"] = CURRENT_DIRECTION_NAMES.get(
        direction, f"UNKNOWN({direction})"
    )
    fet_bitmap = _fet_bitmap_from_summary(summary)
    summary["fet_bitmap"] = fet_bitmap
    summary["fets"] = active_flag_names(fet_bitmap, FET_NAMES)
    return summary


def to_hex(data: bytes) -> str:
    return data.hex(" ").upper()


def from_hex(text: str) -> bytes:
    compact = "".join(text.replace(",", " ").split())
    if not compact:
        return b""
    if len(compact) % 2:
        raise ValueError("hex payload must contain an even number of digits")
    return bytes.fromhex(compact)
