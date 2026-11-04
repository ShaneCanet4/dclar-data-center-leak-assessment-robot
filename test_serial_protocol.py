"""Unit checks for the desktop-to-ESP32 pipe-delimited message format."""

import unittest

from serial_protocol import (
    make_move_message,
    make_ping_message,
    make_segment_message,
    make_task_message,
    parse_board_line,
)


class SerialProtocolTests(unittest.TestCase):
    def test_task_is_pipe_delimited_and_zero_based(self):
        self.assertEqual(
            make_task_message("7", "2", "4", "Zigzag"),
            "TASK|7|2|4|Zigzag",
        )

    def test_move_formats_coordinates(self):
        self.assertEqual(
            make_move_message(3, 1.234, 2, 90, "DRIVE"),
            "MOVE|3|1.23|2.00|90.0|DRIVE",
        )

    def test_ping_has_token(self):
        self.assertEqual(make_ping_message("abc"), "PING|abc")

    def test_rejects_pipe_inside_field(self):
        with self.assertRaises(ValueError):
            make_task_message(1, 0, 0, "zig|zag")

    def test_parse_rx_preserves_robot_message(self):
        self.assertEqual(
            parse_board_line("RX|24:6f:28:aa:bb:cc|DONE|7|2|4\r\n"),
            ("RX", ["24:6f:28:aa:bb:cc", "DONE|7|2|4"]),
        )

    def test_segment_uses_mission_sequence_and_pivot_then_drive_fields(self):
        command = {
            "pivot_angle_degrees": 90,
            "distance_forward_ft": 0.5,
            "target_x_ft": 1.25,
            "target_y_ft": 0.75,
            "global_heading_degrees": 90,
        }
        self.assertEqual(
            make_segment_message("H0001", 2, command),
            "SEG|H0001|2|90.00|0.5000|1.2500|0.7500|90.00",
        )

    def test_move_accepts_auto_heading_and_rejects_infinite_heading(self):
        self.assertEqual(
            make_move_message(4, 1, 2, "AUTO", "DRIVE"),
            "MOVE|4|1.00|2.00|AUTO|DRIVE",
        )
        with self.assertRaises(ValueError):
            make_move_message(4, 1, 2, float("inf"), "DRIVE")


if __name__ == "__main__":
    unittest.main()
