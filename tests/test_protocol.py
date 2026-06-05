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

    def test_hex_helpers(self):
        self.assertEqual(protocol.from_hex("01 02,0A"), b"\x01\x02\x0A")
        self.assertEqual(protocol.to_hex(b"\x01\x02\x0A"), "01 02 0A")


if __name__ == "__main__":
    unittest.main()
