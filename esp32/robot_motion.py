"""Differential-drive geometry for converting path segments to encoder targets.

Positive pivot angles are clockwise/right in the desktop map frame. A right
pivot turns the left wheel forward and the right wheel backward around the
robot's center. This module calculates wheel tick targets only; a board-specific
motor/encoder driver must execute them.
"""

import math


MM_PER_FOOT = 304.8


def _positive_finite(name, value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("%s must be a positive finite number" % name)
    return value


class DifferentialDriveGeometry:
    def __init__(self, wheel_diameter_mm, wheel_track_mm, encoder_ticks_per_wheel_rev):
        self.wheel_diameter_mm = _positive_finite("wheel diameter", wheel_diameter_mm)
        self.wheel_track_mm = _positive_finite("wheel track", wheel_track_mm)
        self.encoder_ticks_per_wheel_rev = _positive_finite(
            "encoder ticks per wheel revolution", encoder_ticks_per_wheel_rev
        )
        self.wheel_circumference_mm = math.pi * self.wheel_diameter_mm

    def millimeters_to_ticks(self, wheel_distance_mm):
        revolutions = float(wheel_distance_mm) / self.wheel_circumference_mm
        return int(round(revolutions * self.encoder_ticks_per_wheel_rev))

    def pivot_ticks(self, clockwise_degrees):
        """Return signed (left, right) tick deltas for an in-place pivot."""
        angle = float(clockwise_degrees)
        if not math.isfinite(angle) or abs(angle) > 180:
            raise ValueError("pivot angle must be finite and within +/-180 degrees")
        # Each wheel travels half the axle-track arc in opposite directions.
        wheel_arc_mm = self.wheel_track_mm * math.radians(angle) / 2
        left_ticks = self.millimeters_to_ticks(wheel_arc_mm)
        return left_ticks, -left_ticks

    def turn_right_ticks(self, degrees):
        """Wheel targets for a clockwise in-place turn (left forward/right back)."""
        degrees = float(degrees)
        if degrees < 0:
            raise ValueError("right-turn angle must be non-negative")
        return self.pivot_ticks(degrees)

    def turn_left_ticks(self, degrees):
        """Wheel targets for a counter-clockwise in-place turn."""
        degrees = float(degrees)
        if degrees < 0:
            raise ValueError("left-turn angle must be non-negative")
        left_ticks, right_ticks = self.pivot_ticks(-degrees)
        return left_ticks, right_ticks

    def forward_ticks(self, distance_ft):
        """Return equal signed wheel tick deltas for a straight drive."""
        distance_ft = float(distance_ft)
        if not math.isfinite(distance_ft) or distance_ft < 0:
            raise ValueError("forward distance must be finite and non-negative")
        ticks = self.millimeters_to_ticks(distance_ft * MM_PER_FOOT)
        return ticks, ticks

    def segment_steps(self, pivot_degrees, distance_ft):
        """Return encoder target pairs in pivot-then-forward order."""
        steps = []
        if abs(float(pivot_degrees)) > 0.01:
            left, right = self.pivot_ticks(pivot_degrees)
            steps.append({"phase": "pivot", "left_ticks": left, "right_ticks": right})
        if float(distance_ft) > 0:
            left, right = self.forward_ticks(distance_ft)
            steps.append({"phase": "drive", "left_ticks": left, "right_ticks": right})
        return steps
