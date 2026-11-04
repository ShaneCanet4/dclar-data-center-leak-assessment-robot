"""Text protocol helpers shared by the desktop app and communication tests."""

import math


def make_ping_message(token):
    token = str(token).strip()
    if not token or "|" in token or "\n" in token:
        raise ValueError("ping token must be a non-empty single field")
    return f"PING|{token}"


def make_task_message(task_id, row, column, pattern):
    task_id = int(task_id)
    row = int(row)
    column = int(column)
    pattern = str(pattern).strip()
    if task_id < 0 or row < 0 or column < 0:
        raise ValueError("task ID, row, and column must be non-negative")
    if not pattern or "|" in pattern or "\n" in pattern:
        raise ValueError("pattern must be a non-empty single field")
    return f"TASK|{task_id}|{row}|{column}|{pattern}"


def make_move_message(task_id, x_ft, y_ft, heading_degrees, instruction):
    task_id = int(task_id)
    x_ft = float(x_ft)
    y_ft = float(y_ft)
    auto_heading = str(heading_degrees).strip().upper() == "AUTO"
    if not auto_heading:
        heading_degrees = float(heading_degrees)
    instruction = str(instruction).strip()
    if task_id < 0:
        raise ValueError("task ID must be non-negative")
    if not instruction or "|" in instruction or "\n" in instruction:
        raise ValueError("instruction must be a non-empty single field")
    if not math.isfinite(x_ft) or not math.isfinite(y_ft):
        raise ValueError("target coordinates must be finite numbers")
    if not auto_heading and not math.isfinite(heading_degrees):
        raise ValueError("heading must be a finite number or AUTO")
    heading_field = "AUTO" if auto_heading else "%.1f" % heading_degrees
    return f"MOVE|{task_id}|{x_ft:.2f}|{y_ft:.2f}|{heading_field}|{instruction}"


def make_segment_message(mission_id, sequence, command):
    """Encode one pivot-then-drive record from movement_command_vector."""
    mission_id = str(mission_id).strip()
    sequence = int(sequence)
    pivot = float(command["pivot_angle_degrees"])
    distance = float(command["distance_forward_ft"])
    target_x = float(command["target_x_ft"])
    target_y = float(command["target_y_ft"])
    heading = float(command["global_heading_degrees"])
    numbers = (pivot, distance, target_x, target_y, heading)
    if not mission_id or "|" in mission_id or "\n" in mission_id:
        raise ValueError("mission ID must be a non-empty single field")
    if sequence < 0:
        raise ValueError("sequence number must be non-negative")
    if not all(math.isfinite(number) for number in numbers):
        raise ValueError("segment coordinates and angles must be finite numbers")
    if abs(pivot) > 180 or distance < 0:
        raise ValueError("pivot must be within +/-180 degrees and distance non-negative")
    return (
        f"SEG|{mission_id}|{sequence}|{pivot:.2f}|{distance:.4f}|"
        f"{target_x:.4f}|{target_y:.4f}|{heading % 360:.2f}"
    )


def parse_board_line(line):
    """Split a bridge line while preserving the robot message after its prefix."""
    parts = str(line).strip().split("|", 2)
    if not parts or not parts[0]:
        return "", []
    return parts[0], parts[1:]
