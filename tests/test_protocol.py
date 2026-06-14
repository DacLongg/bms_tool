import struct
import unittest

from bms_tool_app import protocol


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


if __name__ == "__main__":
    unittest.main()
