"""Robot-side ESP-NOW receiver and differential-drive command translator.

Copy this file and robot_motion.py onto the robot ESP32. Commands are sent by
the main-control ESP32 as pipe-delimited ASCII. This file intentionally refuses
to move until wheel geometry and a real encoder-aware motor_driver.py are set
up for the robot's motor board.
"""

import network
import espnow
import math
import sys
from time import sleep_ms

from robot_motion import DifferentialDriveGeometry


# Match the main-control board's channel. The main board learns this ESP32's
# MAC from its own configured ROBOT_MAC; this robot learns the main peer from
# the sender MAC on the first received packet.
WIFI_CHANNEL = 1
MAX_RADIO_PAYLOAD = 240

# TODO: Measure these on the assembled robot. Do not substitute the 7 x 5 in
# body dimensions: wheel_track_mm is the distance between left/right wheel
# contact centerlines, and wheel diameter must include the loaded tire radius.
WHEEL_DIAMETER_MM = None
WHEEL_TRACK_MM = None
ENCODER_TICKS_PER_WHEEL_REV = None
MAX_WHEEL_SPEED_MM_S = None
MOTOR_TIMEOUT_MS = 5000
START_X_FT = 0.5
START_Y_FT = 0.5
START_HEADING_DEGREES = 0.0

try:
    import motor_driver
except ImportError:
    motor_driver = None


def make_geometry():
    if None in (WHEEL_DIAMETER_MM, WHEEL_TRACK_MM, ENCODER_TICKS_PER_WHEEL_REV):
        return None
    return DifferentialDriveGeometry(
        WHEEL_DIAMETER_MM,
        WHEEL_TRACK_MM,
        ENCODER_TICKS_PER_WHEEL_REV,
    )


def send_reply(radio, peer_mac, message):
    encoded = message.encode()
    if len(encoded) > MAX_RADIO_PAYLOAD:
        raise ValueError("reply exceeds ESP-NOW packet limit")
    radio.send(peer_mac, encoded)
    sys.stdout.write("TX " + message + "\n")


def stop_motors():
    if motor_driver is not None and hasattr(motor_driver, "stop"):
        motor_driver.stop()


def run_segment(segment, geometry):
    """Translate one SEG into encoder targets and execute pivot, then drive.

    The motor_driver module must implement:
      move_wheels(left_ticks, right_ticks, max_speed_mm_s, timeout_ms)
      stop()
    move_wheels must be closed-loop, stop on timeout, and raise on encoder or
    motor faults. This file supplies wheel targets, not board-specific pins.
    """
    if geometry is None:
        raise RuntimeError("MOTION_CONFIG_REQUIRED")
    if motor_driver is None or not hasattr(motor_driver, "move_wheels"):
        raise RuntimeError("MOTOR_DRIVER_MISSING")
    if MAX_WHEEL_SPEED_MM_S is None or MAX_WHEEL_SPEED_MM_S <= 0:
        raise RuntimeError("MAX_SPEED_NOT_CONFIGURED")

    steps = geometry.segment_steps(
        segment["pivot_degrees"], segment["distance_ft"]
    )
    motor_driver.stop()
    for step in steps:
        motor_driver.move_wheels(
            step["left_ticks"],
            step["right_ticks"],
            MAX_WHEEL_SPEED_MM_S,
            MOTOR_TIMEOUT_MS,
        )
    motor_driver.stop()
    return steps


def parse_segment(text):
    """Parse SEG|mission|seq|pivot_deg|distance_ft|x|y|heading."""
    fields = text.split("|")
    if len(fields) != 8 or fields[0] != "SEG":
        raise ValueError("EXPECTED_SEGMENT_FORMAT")
    mission_id = fields[1]
    sequence = int(fields[2])
    if not mission_id or sequence < 0:
        raise ValueError("BAD_MISSION_OR_SEQUENCE")
    segment = {
        "mission_id": mission_id,
        "sequence": sequence,
        "pivot_degrees": float(fields[3]),
        "distance_ft": float(fields[4]),
        "target_x_ft": float(fields[5]),
        "target_y_ft": float(fields[6]),
        "target_heading_degrees": float(fields[7]),
    }
    for value in segment.values():
        if isinstance(value, float) and (value != value or abs(value) == float("inf")):
            raise ValueError("NON_FINITE_SEGMENT_VALUE")
    if abs(segment["pivot_degrees"]) > 180 or segment["distance_ft"] < 0:
        raise ValueError("SEGMENT_OUT_OF_RANGE")
    return segment


def parse_move(text, current_pose):
    """Translate MOVE|id|x|y|heading-or-AUTO|DRIVE/TURN to a segment."""
    fields = text.split("|")
    if len(fields) != 6 or fields[0] != "MOVE":
        raise ValueError("EXPECTED_MOVE_FORMAT")
    move_id = fields[1]
    target_x = float(fields[2])
    target_y = float(fields[3])
    instruction = fields[5].strip().upper()
    current_x, current_y, current_heading = current_pose

    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)
    if instruction == "TURN":
        if fields[4].upper() == "AUTO":
            raise ValueError("TURN_REQUIRES_HEADING")
        target_heading = float(fields[4]) % 360
        distance = 0.0
    elif instruction == "DRIVE":
        if distance < 1e-6:
            raise ValueError("DRIVE_TARGET_EQUALS_CURRENT_POSITION")
        target_heading = math.degrees(math.atan2(dy, dx)) % 360
        if fields[4].upper() != "AUTO":
            requested_heading = float(fields[4]) % 360
            mismatch = (requested_heading - target_heading + 180) % 360 - 180
            if abs(mismatch) > 3:
                raise ValueError("HEADING_DOES_NOT_POINT_AT_TARGET")
    else:
        raise ValueError("INSTRUCTION_MUST_BE_DRIVE_OR_TURN")

    pivot = (target_heading - current_heading + 180) % 360 - 180
    return {
        "mission_id": move_id,
        "sequence": 0,
        "pivot_degrees": pivot,
        "distance_ft": distance,
        "target_x_ft": target_x,
        "target_y_ft": target_y,
        "target_heading_degrees": target_heading,
    }


def handle_radio_message(radio, sender_mac, raw_payload, geometry, last_done):
    global robot_pose
    try:
        text = raw_payload.decode().strip()
    except Exception:
        send_reply(radio, sender_mac, "ERR|RADIO|NON_TEXT")
        return last_done

    try:
        if text.startswith("RADIO_PING|"):
            token = text.split("|", 1)[1]
            send_reply(radio, sender_mac, "RADIO_PONG|" + token)
            return last_done

        if text.startswith("TASK|"):
            fields = text.split("|")
            task_id = fields[1] if len(fields) > 1 else "?"
            send_reply(radio, sender_mac, "ERR|%s|PATH_SEGMENTS_REQUIRED" % task_id)
            return last_done

        if text.startswith("MOVE|"):
            segment = parse_move(text, robot_pose)
        else:
            segment = parse_segment(text)
        key = (segment["mission_id"], segment["sequence"])
        if key == last_done:
            send_reply(radio, sender_mac, "DONE|%s|%d" % key)
            return last_done

        # Confirm that the radio delivered a well-formed, identified segment.
        send_reply(radio, sender_mac, "RECEIVED|%s|%d" % key)
        steps = run_segment(segment, geometry)
        robot_pose = (
            segment["target_x_ft"],
            segment["target_y_ft"],
            segment["target_heading_degrees"],
        )
        # Report the generated wheel deltas so the computer log can verify the
        # right/left pivot convention before it advances to the next segment.
        plan_fields = []
        for step in steps:
            plan_fields.append("%s:%d:%d" % (
                step["phase"], step["left_ticks"], step["right_ticks"]
            ))
        send_reply(
            radio,
            sender_mac,
            "PLAN|%s|%d|%s" % (key[0], key[1], ",".join(plan_fields)),
        )
        send_reply(radio, sender_mac, "DONE|%s|%d" % key)
        return key
    except Exception as error:
        stop_motors()
        reason = str(error).replace("|", "/").replace(" ", "_")
        fields = text.split("|")
        mission = fields[1] if len(fields) > 1 else "?"
        sequence = fields[2] if len(fields) > 2 else "?"
        send_reply(radio, sender_mac, "ERR|%s|%s|%s" % (mission, sequence, reason))
        return last_done


# Bring up Wi-Fi station mode and ESP-NOW on the shared radio channel.
sta = network.WLAN(network.STA_IF)
sta.active(True)
try:
    sta.disconnect()
except Exception:
    pass
sta.config(channel=WIFI_CHANNEL)

radio = espnow.ESPNow()
radio.active(True)
geometry = make_geometry()
last_done = None
robot_pose = (START_X_FT, START_Y_FT, START_HEADING_DEGREES)

sys.stdout.write("ROBOT_READY|%d|%s\n" % (
    WIFI_CHANNEL,
    ":".join("%02x" % value for value in sta.config("mac")),
))

while True:
    received = radio.recv(0)
    # recv(0) returns (None, None) when no packet is ready on MicroPython 1.28.
    if received and received[0] is not None and received[1] is not None:
        sender_mac, raw_payload = received
        try:
            radio.add_peer(sender_mac)
        except Exception:
            # The main controller may already be registered as an ESP-NOW peer.
            pass
        sys.stdout.write("RX " + raw_payload.decode(errors="replace") + "\n")
        last_done = handle_radio_message(
            radio, sender_mac, raw_payload, geometry, last_done
        )
    sleep_ms(10)
