"""Host-side checks for differential-drive pivot and distance translation."""

import unittest

from esp32.robot_motion import DifferentialDriveGeometry


class DifferentialDriveTests(unittest.TestCase):
    def setUp(self):
        self.geometry = DifferentialDriveGeometry(
            wheel_diameter_mm=65,
            wheel_track_mm=120,
            encoder_ticks_per_wheel_rev=360,
        )

    def test_right_turn_drives_left_forward_and_right_backward(self):
        left_ticks, right_ticks = self.geometry.turn_right_ticks(90)
        self.assertGreater(left_ticks, 0)
        self.assertLess(right_ticks, 0)
        self.assertEqual(left_ticks, -right_ticks)

    def test_left_turn_reverses_right_turn_wheel_directions(self):
        left_ticks, right_ticks = self.geometry.turn_left_ticks(90)
        self.assertLess(left_ticks, 0)
        self.assertGreater(right_ticks, 0)
        self.assertEqual(left_ticks, -right_ticks)

    def test_forward_drive_moves_both_wheels_forward_equally(self):
        left_ticks, right_ticks = self.geometry.forward_ticks(1)
        self.assertGreater(left_ticks, 0)
        self.assertEqual(left_ticks, right_ticks)

    def test_segment_executes_pivot_before_forward_drive(self):
        steps = self.geometry.segment_steps(90, 0.5)
        self.assertEqual([step["phase"] for step in steps], ["pivot", "drive"])
        self.assertGreater(steps[0]["left_ticks"], 0)
        self.assertLess(steps[0]["right_ticks"], 0)

    def test_refuses_zero_or_negative_wheel_geometry(self):
        with self.assertRaises(ValueError):
            DifferentialDriveGeometry(0, 120, 360)


if __name__ == "__main__":
    unittest.main()
