import struct
import unittest

from bms_tool_app import protocol
from bms_tool_app.serial_client import BmsSerialClient


class ProtocolTests(unittest.TestCase):
    def test_crc16_matches_known_modbus_vector(self):
        self.assertEqual(protocol.crc16_modbus(b"123456789"), 0x4B37)

    def test_build_and_parse_frame(self):
        frame = protocol.build_frame(protocol.CMD_PING, b"abc")
        parser = protocol.FrameParser()
        frames = parser.feed(frame[:2]) + parser.feed(frame[2:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].command, protocol.CMD_PING)
        self.assertEqual(frames[0].payload, b"abc")

    def test_default_parser_rejects_payload_over_protocol_limit(self):
        raw = bytes([protocol.SOF0, protocol.SOF1, protocol.CMD_PING, 161])
        parser = protocol.FrameParser()
        self.assertEqual(parser.feed(raw + bytes(163)), [])

    def test_decode_response_status_ok(self):
        response = protocol.Frame(protocol.CMD_PING | protocol.RESPONSE_FLAG, b"\x00abc")
        self.assertEqual(protocol.decode_response(protocol.CMD_PING, response), b"abc")

    def test_decode_response_status_error(self):
        response = protocol.Frame(
            protocol.CMD_READ_SUMMARY | protocol.RESPONSE_FLAG,
            bytes([protocol.STATUS_INTERNAL_ERROR]),
        )
        with self.assertRaises(protocol.BmsStatusError) as ctx:
            protocol.decode_response(protocol.CMD_READ_SUMMARY, response)
        self.assertEqual(ctx.exception.status, protocol.STATUS_INTERNAL_ERROR)

    def test_parse_cells(self):
        data = bytes([3]) + struct.pack("<HHH", 3700, 3710, 3695)
        parsed = protocol.parse_cells(data)
        self.assertEqual(parsed["cell_count"], 3)
        self.assertEqual(parsed["cell_voltages_mV"], [3700, 3710, 3695])
        self.assertEqual(parsed["delta_cell_voltage_mV"], 15)

    def test_parse_faults(self):
        data = struct.pack("<HBBI", (1 << 0) | (1 << 7), 0x02, 1, 123)
        parsed = protocol.parse_faults(data)
        self.assertIn("Cell over voltage", parsed["faults"])
        self.assertIn("Short circuit", parsed["faults"])
        self.assertIn("DDSG pin active", parsed["gate_signals"])
        self.assertTrue(parsed["alert_active"])
        self.assertEqual(parsed["alert_counter"], 123)

    def test_parse_protection_event(self):
        parsed = protocol.parse_protection_event(bytes([0x08]))
        self.assertEqual(parsed["reason"], 0x08)
        self.assertEqual(parsed["reason_name"], "Short circuit")
        self.assertEqual(parsed["fault_name"], "Short circuit")
        self.assertEqual(parsed["raw_hex"], "08")

    def test_all_documented_commands_are_named(self):
        commands = {
            0x01, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
            0x18, 0x19, 0x20, 0x21, 0x22, 0x30, 0x31, 0x40,
        }
        self.assertEqual(set(protocol.COMMAND_NAMES), commands)

    def test_documented_frame_examples(self):
        self.assertEqual(
            protocol.to_hex(protocol.build_frame(protocol.CMD_READ_CURRENT)),
            "AA 55 14 00 0E B0",
        )
        self.assertEqual(
            protocol.to_hex(
                protocol.build_frame(protocol.CMD_SET_BALANCING, b"\x00")
            ),
            "AA 55 31 01 00 21 9F",
        )

    def test_parse_dedicated_read_commands(self):
        self.assertEqual(
            protocol.parse_current(struct.pack("<i", -1000))["current_mA"],
            -1000,
        )
        status = protocol.parse_status(struct.pack("<BBH", 75, 0x35, 0x41))
        self.assertEqual(status["soc_percent"], 75)
        self.assertTrue(status["charging"])
        self.assertTrue(status["balancing_active"])
        self.assertTrue(status["balancing_enabled"])
        self.assertIn("Short circuit", status["fault_flag_names"])
        self.assertEqual(
            protocol.parse_pack_voltage(struct.pack("<I", 37000))["pack_voltage_mV"],
            37000,
        )
        self.assertEqual(protocol.parse_cell_count(b"\x0A")["cell_count"], 10)
        temperature = protocol.parse_temperature(struct.pack("<h", -50))
        self.assertEqual(temperature["temperature_deciC"], -50)
        self.assertEqual(temperature["temperature_C"], -5.0)
        self.assertEqual(
            protocol.parse_cycle_count(struct.pack("<I", 123))["completed_cycle_count"],
            123,
        )

    def test_parse_current_tracking_summary_v1_3(self):
        data = bytearray(protocol.RAW_TRACKING_SIZE)
        data[0:4] = bytes([1, 1, 1, 2])
        data[4:14] = bytes(range(10))
        struct.pack_into("<10H", data, 14, *range(100, 110))
        struct.pack_into("<10H", data, 34, *range(3700, 3710))
        struct.pack_into("<HHHHH", data, 54, 3700, 3709, 3704, 9, 37040)
        struct.pack_into("<i", data, 64, -1234)
        struct.pack_into("<hh", data, 68, 25, 26)
        data[72:79] = bytes([0, 1, 1, 1, 1, 0, 0])
        struct.pack_into("<H", data, 80, 0x1234)
        data[82] = 1
        data[89] = 1
        data[92:98] = bytes([0, 1, 1, 0, 1, 1])
        struct.pack_into("<I", data, 100, 42)
        data[104:107] = bytes([0, 1, 1])
        struct.pack_into("<H", data, 108, 37100)
        data[110] = 1
        struct.pack_into("<H", data, 112, 0x0155)
        struct.pack_into("<II", data, 116, 30, 999999)

        parsed = protocol.parse_summary(bytes(data))
        self.assertEqual(parsed["summary_format"], "raw_tracking_v1_3")
        self.assertEqual(parsed["parsed_length"], 124)
        self.assertEqual(parsed["state_name"], "NORMAL")
        self.assertEqual(parsed["current_direction_name"], "DISCHARGE")
        self.assertEqual(parsed["cell_voltages_mV"], list(range(3700, 3710)))
        self.assertEqual(parsed["pack_voltage_mV"], 37040)
        self.assertEqual(parsed["current_mA"], -1234)
        self.assertEqual(parsed["temperature_C"], [25, 26])
        self.assertIn("Cell over voltage", parsed["faults"])
        self.assertIn("Short circuit", parsed["faults"])
        self.assertEqual(parsed["gate_signal_bitmap"], 0x01)
        self.assertEqual(parsed["balance_mask"], 0x0155)
        self.assertEqual(parsed["completed_cycle_count"], 30)
        self.assertEqual(parsed["current_calibration_gain_ppm"], 999999)

    def test_balancing_payload_and_response(self):
        self.assertEqual(protocol.make_balancing_payload(False), b"\x00")
        self.assertEqual(protocol.make_balancing_payload(True), b"\x01")
        self.assertEqual(protocol.parse_balancing(b"\x01"), {"mode": 1, "enabled": True})
        with self.assertRaises(ValueError):
            protocol.make_balancing_payload(2)
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_balancing(b"\x02")

    def test_fixed_payload_parsers_reject_wrong_length(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_current(b"\x00")
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_otp_status(bytes(26))

    def test_parse_raw_tracking_summary_with_short_enums(self):
        data = bytearray(protocol.RAW_TRACKING_SHORT_ENUM_SIZE)
        data[0] = 1
        data[1] = 1
        data[2] = 1
        data[3] = 2

        cell_base = 4
        data[cell_base : cell_base + 10] = bytes(range(10))
        struct.pack_into("<10H", data, cell_base + 10, *range(100, 110))
        struct.pack_into("<10H", data, cell_base + 30, *range(3700, 3710))
        struct.pack_into("<HHHH", data, cell_base + 50, 3700, 3709, 3704, 9)

        struct.pack_into("<HHH", data, 62, 37040, 37000, 77)
        struct.pack_into("<i", data, 68, -1234)
        struct.pack_into("<hh", data, 72, 25, 26)
        data[76] = 1
        data[78] = 1
        data[79] = 1
        data[80] = 1
        data[82] = 1
        struct.pack_into("<H", data, 84, 0x1234)
        data[86] = 1
        data[93] = 1
        data[96] = 1
        data[98] = 1
        data[100] = 1
        data[101] = 1
        struct.pack_into("<I", data, 104, 42)
        data[108] = 1
        data[110] = 1
        struct.pack_into("<H", data, 112, 37100)
        data[114] = 1
        struct.pack_into("<H", data, 116, 0x0155)
        struct.pack_into("<Q", data, 120, 1000)
        struct.pack_into("<Q", data, 128, 2000)
        struct.pack_into("<III", data, 136, 10, 30, 999999)

        parsed = protocol.parse_summary(bytes(data))
        self.assertEqual(parsed["summary_format"], "raw_tracking_short_enums")
        self.assertEqual(parsed["payload_length"], 152)
        self.assertEqual(parsed["parsed_length"], 148)
        self.assertEqual(parsed["state_name"], "NORMAL")
        self.assertEqual(parsed["current_direction_name"], "DISCHARGE")
        self.assertEqual(parsed["cell_voltages_mV"], list(range(3700, 3710)))
        self.assertEqual(parsed["stack_voltage_mV"], 37040)
        self.assertEqual(parsed["pack_voltage_mV"], 37000)
        self.assertEqual(parsed["current_mA"], -1234)
        self.assertEqual(parsed["temperature_C"], [25, 26])
        self.assertIn("Cell over voltage", parsed["faults"])
        self.assertIn("Short circuit", parsed["faults"])
        self.assertEqual(parsed["gate_signal_bitmap"], 0x01)
        self.assertEqual(parsed["balance_mask"], 0x0155)
        self.assertEqual(parsed["charge_throughput_mAh"], 10)
        self.assertEqual(parsed["equivalent_cycle_milliCycles"], 30)
        self.assertEqual(parsed["current_calibration_gain_ppm"], 999999)
        self.assertNotIn("discharge_throughput_mAh", parsed)

    def test_parse_current_calibration_result(self):
        data = struct.pack("<iII", -1200, 25000, 987654)
        parsed = protocol.parse_current_calibration_result(
            data,
            status=protocol.STATUS_OK,
            actual_ma=-1000,
        )
        self.assertEqual(parsed["status_name"], "OK")
        self.assertEqual(parsed["actual_mA"], -1000)
        self.assertEqual(parsed["measured_mA"], -1200)
        self.assertEqual(parsed["deviation_ppm"], 25000)
        self.assertEqual(parsed["new_gain_ppm"], 987654)
        self.assertNotIn("old_gain_ppm", parsed)

    def test_hex_helpers(self):
        self.assertEqual(protocol.from_hex("01 02,0A"), b"\x01\x02\x0A")
        self.assertEqual(protocol.to_hex(b"\x01\x02\x0A"), "01 02 0A")


class SerialClientApiTests(unittest.TestCase):
    def test_dedicated_api_methods_use_documented_commands(self):
        responses = {
            protocol.CMD_READ_CURRENT: struct.pack("<i", -321),
            protocol.CMD_READ_STATUS: struct.pack("<BBH", 50, 0x20, 0),
            protocol.CMD_READ_PACK_VOLTAGE: struct.pack("<I", 37000),
            protocol.CMD_READ_CELL_COUNT: b"\x0A",
            protocol.CMD_READ_TEMPERATURE: struct.pack("<h", 290),
            protocol.CMD_READ_CYCLE_COUNT: struct.pack("<I", 7),
            protocol.CMD_SET_BALANCING: b"\x01",
        }
        calls = []
        client = BmsSerialClient()

        def fake_request(command, payload=b"", timeout_s=1.0):
            calls.append((command, payload, timeout_s))
            return responses[command]

        client.request = fake_request
        self.assertEqual(client.read_current()["current_mA"], -321)
        self.assertTrue(client.read_status()["balancing_enabled"])
        self.assertEqual(client.read_pack_voltage()["pack_voltage_mV"], 37000)
        self.assertEqual(client.read_cell_count()["cell_count"], 10)
        self.assertEqual(client.read_temperature()["temperature_C"], 29.0)
        self.assertEqual(client.read_cycle_count()["completed_cycle_count"], 7)
        self.assertTrue(client.set_balancing(True)["enabled"])
        self.assertEqual(calls[-1][:2], (protocol.CMD_SET_BALANCING, b"\x01"))

    def test_ping_rejects_160_byte_echo_payload(self):
        client = BmsSerialClient()
        with self.assertRaises(protocol.PayloadTooLargeError):
            client.ping(bytes(protocol.MAX_PAYLOAD_SIZE))


if __name__ == "__main__":
    unittest.main()
