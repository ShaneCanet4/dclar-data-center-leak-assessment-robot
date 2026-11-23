"""Robot-side ESP-NOW receiver for motor jogs and scheduled tile paths.

Copy this file, motor_driver.py, and motor_test_protocol.py to the robot ESP32
root (the host-side source for each is in this esp32 folder).
The MOTOR_TEST path is a short manual jog. Planner paths use a declared tile
mission followed by one SEG packet at a time; the robot reports DONE only after
the calibrated time schedule for that segment finishes, and TILE_DONE after
all declared segments finish. This is open-loop confirmation, not position
feedback. Leave calibration values unset until measured on the assembled robot.
"""

import network
import espnow
import math
import sys
from time import sleep_ms, ticks_ms, ticks_diff

from motor_test_protocol import (
    parse_motor_stop_message,
    parse_motor_test_message,
)


# Match the main-control board's channel. The main board learns this ESP32's
# MAC from its own configured ROBOT_MAC; this robot learns the main peer from
# the sender MAC on the first received packet.
WIFI_CHANNEL = 1
MAX_RADIO_PAYLOAD = 240

# Measure these on the assembled robot, at PATH_DUTY_U16 and the intended load.
# A segment acknowledgement means this schedule elapsed; there are no encoders.
DRIVE_SPEED_MM_S = None
TURN_RATE_DEG_S = None
PATH_DUTY_U16 = 19660
MAX_PATH_STEP_MS = 8000
MAX_SEGMENT_DISTANCE_FT = 2.5
MAX_TILE_SEGMENTS = 300
START_X_FT = 0.5
START_Y_FT = 0.5
START_HEADING_DEGREES = 0.0

try:
    import motor_driver
except Exception as error:
    motor_driver = None
    sys.stdout.write("MOTOR_DRIVER_IMPORT_ERROR|%s\n" % str(error).replace("|", "/"))

active_motor_test = None
active_tile_mission = None
active_path_job = None
last_completed_tile = None
robot_pose_valid = True


def send_reply(radio, peer_mac, message):
    encoded = message.encode()
    if len(encoded) > MAX_RADIO_PAYLOAD:
        raise ValueError("reply exceeds ESP-NOW packet limit")
    radio.send(peer_mac, encoded)
    sys.stdout.write("TX " + message + "\n")


def stop_motors():
    if motor_driver is not None and hasattr(motor_driver, "stop"):
        motor_driver.stop()


def handle_motor_test_message(radio, peer_mac, text):
    """Start or stop one short, acknowledged open-loop motor jog."""
    global active_motor_test
    if text.startswith("MOTOR_STOP|"):
        token = "?"
        try:
            token = parse_motor_stop_message(text)
            if active_motor_test is None:
                stop_motors()
                send_reply(radio, peer_mac, "MOTOR_TEST_STOPPED|%s|0" % token)
                return
            if token != active_motor_test["test_id"]:
                send_reply(radio, peer_mac, "MOTOR_TEST_ERR|%s|ID_MISMATCH" % token)
                return
            elapsed = ticks_diff(ticks_ms(), active_motor_test["started_ms"])
            stop_motors()
            active_motor_test = None
            send_reply(radio, peer_mac, "MOTOR_TEST_STOPPED|%s|%d" % (token, elapsed))
        except Exception as error:
            reason = str(error).replace("|", "/").replace(" ", "_")
            send_reply(radio, peer_mac, "MOTOR_TEST_ERR|%s|%s" % (token, reason))
        return

    token = "?"
    motors_touched = False
    try:
        command = parse_motor_test_message(text)
        token = command["test_id"]
        if active_motor_test is not None:
            send_reply(radio, peer_mac, "MOTOR_TEST_ERR|%s|MOTOR_TEST_BUSY" % token)
            return
        if motor_driver is None or not hasattr(motor_driver, "set_motors"):
            raise RuntimeError("MOTOR_DRIVER_MISSING")
        if not hasattr(motor_driver, "MAX_DUTY_U16"):
            raise RuntimeError("MOTOR_DRIVER_CONFIG_INVALID")
        if command["duty_u16"] > motor_driver.MAX_DUTY_U16:
            raise RuntimeError("DUTY_ABOVE_DRIVER_LIMIT")

        motors_touched = True
        stop_motors()
        motor_driver.set_motors(
            command["duty_u16"], command["direction_a"],
            command["duty_u16"], command["direction_b"],
        )
        active_motor_test = {
            "test_id": token,
            "started_ms": ticks_ms(),
            "duration_ms": command["duration_ms"],
            "peer_mac": peer_mac,
        }
        send_reply(
            radio, peer_mac,
            "MOTOR_TEST_STARTED|%s|%d" % (token, command["duration_ms"]),
        )
    except Exception as error:
        if motors_touched:
            stop_motors()
            if active_motor_test is not None and active_motor_test["test_id"] == token:
                active_motor_test = None
        reason = str(error).replace("|", "/").replace(" ", "_")
        send_reply(radio, peer_mac, "MOTOR_TEST_ERR|%s|%s" % (token, reason))


def service_motor_test(radio):
    """Stop a jog at its deadline, even if no STOP packet is sent."""
    global active_motor_test
    if active_motor_test is None:
        return
    elapsed = ticks_diff(ticks_ms(), active_motor_test["started_ms"])
    if elapsed >= active_motor_test["duration_ms"]:
        token = active_motor_test["test_id"]
        peer_mac = active_motor_test["peer_mac"]
        stop_motors()
        active_motor_test = None
        send_reply(radio, peer_mac, "MOTOR_TEST_DONE|%s|%d" % (token, elapsed))


def parse_mission_start(text):
    fields = text.split("|")
    if len(fields) != 6 or fields[0] != "MISSION_START":
        raise ValueError("EXPECTED_MISSION_START_FORMAT")
    mission_id = fields[1]
    row, column, count = int(fields[2]), int(fields[3]), int(fields[5])
    pattern = fields[4]
    if (not mission_id or pattern not in ("Zigzag", "Circle_spiral")
            or row < 0 or column < 0):
        raise ValueError("BAD_TILE_MISSION_FIELDS")
    if count < 1 or count > MAX_TILE_SEGMENTS:
        raise ValueError("TILE_SEGMENT_COUNT_OUT_OF_RANGE")
    return {"mission_id": mission_id, "row": row, "column": column,
            "pattern": pattern, "segment_count": count}


def path_calibration_error():
    if DRIVE_SPEED_MM_S is None or TURN_RATE_DEG_S is None:
        return "PATH_CALIBRATION_REQUIRED"
    for value in (DRIVE_SPEED_MM_S, TURN_RATE_DEG_S):
        if value != value or abs(value) == float("inf") or value <= 0:
            return "PATH_CALIBRATION_INVALID"
    if motor_driver is None or not hasattr(motor_driver, "set_motors"):
        return "MOTOR_DRIVER_MISSING"
    if not hasattr(motor_driver, "MAX_DUTY_U16"):
        return "MOTOR_DRIVER_CONFIG_INVALID"
    if PATH_DUTY_U16 < 1 or PATH_DUTY_U16 > motor_driver.MAX_DUTY_U16:
        return "PATH_DUTY_OUT_OF_RANGE"
    return None


def schedule_segment(segment):
    """Create bounded, open-loop pivot and drive steps for one waypoint."""
    issue = path_calibration_error()
    if issue:
        raise RuntimeError(issue)
    if segment["distance_ft"] > MAX_SEGMENT_DISTANCE_FT:
        raise ValueError("SEGMENT_DISTANCE_OUT_OF_RANGE")
    steps = []
    pivot = segment["pivot_degrees"]
    if abs(pivot) > 0.01:
        duration = int(round(abs(pivot) * 1000.0 / TURN_RATE_DEG_S))
        if duration > MAX_PATH_STEP_MS:
            raise ValueError("PIVOT_DURATION_OUT_OF_RANGE")
        direction = 1 if pivot > 0 else -1
        steps.append((max(1, duration), direction, -direction))
    distance_mm = segment["distance_ft"] * 304.8
    if distance_mm > 0:
        duration = int(round(distance_mm * 1000.0 / DRIVE_SPEED_MM_S))
        if duration > MAX_PATH_STEP_MS:
            raise ValueError("DRIVE_DURATION_OUT_OF_RANGE")
        steps.append((max(1, duration), 1, 1))
    if not steps:
        raise ValueError("EMPTY_SEGMENT")
    return steps


def start_path_step(job):
    duration_ms, left_direction, right_direction = job["steps"][job["step_index"]]
    motor_driver.set_motors(
        PATH_DUTY_U16, left_direction, PATH_DUTY_U16, right_direction
    )
    job["step_started_ms"] = ticks_ms()
    job["step_duration_ms"] = duration_ms


def service_path_execution(radio):
    """Advance the non-blocking timed path and emit segment/tile receipts."""
    global active_path_job, active_tile_mission, last_completed_tile, robot_pose
    job = active_path_job
    if job is None:
        return
    if ticks_diff(ticks_ms(), job["step_started_ms"]) < job["step_duration_ms"]:
        return
    motor_driver.stop()
    job["step_index"] += 1
    if job["step_index"] < len(job["steps"]):
        start_path_step(job)
        return

    mission = active_tile_mission
    segment = job["segment"]
    robot_pose = (segment["target_x_ft"], segment["target_y_ft"],
                  segment["target_heading_degrees"])
    sequence = segment["sequence"]
    if mission["next_sequence"] + 1 == mission["segment_count"]:
        expected_x = mission["column"] + 0.5
        expected_y = mission["row"] + 0.5
        if (abs(robot_pose[0] - expected_x) > 0.03
                or abs(robot_pose[1] - expected_y) > 0.03):
            robot_pose_valid = False
            active_path_job = None
            active_tile_mission = None
            send_reply(radio, job["peer_mac"], "ERR|%s|%d|TILE_ENDPOINT_MISMATCH" % (
                mission["mission_id"], sequence
            ))
            return
    mission["next_sequence"] += 1
    active_path_job = None
    send_reply(radio, job["peer_mac"], "DONE|%s|%d|%.4f|%.4f|%.2f" % (
        mission["mission_id"], sequence, robot_pose[0], robot_pose[1],
        robot_pose[2],
    ))
    if mission["next_sequence"] == mission["segment_count"]:
        completed = (mission["mission_id"], mission["row"], mission["column"],
                     mission["segment_count"])
        last_completed_tile = completed
        send_reply(radio, job["peer_mac"], "TILE_DONE|%s|%d|%d|%d" % completed)
        active_tile_mission = None


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
    global robot_pose, robot_pose_valid, active_tile_mission, active_path_job
    try:
        text = raw_payload.decode().strip()
    except Exception:
        send_reply(radio, sender_mac, "ERR|RADIO|NON_TEXT")
        return last_done

    try:
        if text.startswith("RADIO_PING|"):
            token = text.split("|", 1)[1]
            path_status = "PATH_READY" if path_calibration_error() is None else "PATH_NOT_CONFIGURED"
            pose_status = "POSE_VALID" if robot_pose_valid else "POSE_UNKNOWN"
            send_reply(radio, sender_mac, "RADIO_PONG|%s|%s|%s|%.4f|%.4f|%.2f" % (
                token, path_status, pose_status, robot_pose[0], robot_pose[1], robot_pose[2]
            ))
            return last_done

        if text.startswith("MISSION_STOP|"):
            fields = text.split("|")
            mission_id = fields[1] if len(fields) == 2 else "?"
            if active_tile_mission is not None and active_tile_mission["mission_id"] == mission_id:
                if active_path_job is not None:
                    robot_pose_valid = False
                stop_motors()
                active_path_job = None
                active_tile_mission = None
                send_reply(radio, sender_mac, "MISSION_STOPPED|" + mission_id)
            else:
                send_reply(radio, sender_mac, "ERR|%s|0|MISSION_ID_MISMATCH" % mission_id)
            return last_done

        if text.startswith("MOTOR_TEST|") or text.startswith("MOTOR_STOP|"):
            if active_tile_mission is not None:
                send_reply(radio, sender_mac, "MOTOR_TEST_ERR|?|PATH_BUSY")
                return last_done
            if text.startswith("MOTOR_TEST|"):
                robot_pose_valid = False
            handle_motor_test_message(radio, sender_mac, text)
            return last_done

        if active_motor_test is not None:
            send_reply(radio, sender_mac, "ERR|MOTOR_TEST|BUSY")
            return last_done

        if text.startswith("TASK|"):
            fields = text.split("|")
            task_id = fields[1] if len(fields) > 1 else "?"
            send_reply(radio, sender_mac, "ERR|%s|PATH_SEGMENTS_REQUIRED" % task_id)
            return last_done

        if text.startswith("MISSION_START|"):
            mission = parse_mission_start(text)
            if active_motor_test is not None:
                send_reply(radio, sender_mac, "ERR|%s|0|MOTOR_TEST_BUSY" % mission["mission_id"])
                return last_done
            issue = path_calibration_error()
            if issue:
                send_reply(radio, sender_mac, "ERR|%s|0|%s" % (
                    mission["mission_id"], issue
                ))
                return last_done
            if active_path_job is not None:
                if (active_tile_mission is not None
                        and active_tile_mission["mission_id"] == mission["mission_id"]
                        and active_tile_mission["row"] == mission["row"]
                        and active_tile_mission["column"] == mission["column"]
                        and active_tile_mission["pattern"] == mission["pattern"]
                        and active_tile_mission["segment_count"] == mission["segment_count"]):
                    send_reply(radio, sender_mac, "MISSION_READY|%s|%d" % (
                        mission["mission_id"], mission["segment_count"]
                    ))
                    return last_done
                send_reply(radio, sender_mac, "ERR|%s|0|MISSION_BUSY" % mission["mission_id"])
                return last_done
            if active_tile_mission is not None:
                if (active_tile_mission["mission_id"] == mission["mission_id"]
                        and active_tile_mission["row"] == mission["row"]
                        and active_tile_mission["column"] == mission["column"]
                        and active_tile_mission["pattern"] == mission["pattern"]
                        and active_tile_mission["segment_count"] == mission["segment_count"]):
                    send_reply(radio, sender_mac, "MISSION_READY|%s|%d" % (
                        mission["mission_id"], mission["segment_count"]
                    ))
                else:
                    send_reply(radio, sender_mac, "ERR|%s|0|MISSION_BUSY" % mission["mission_id"])
                return last_done
            if (last_completed_tile is not None
                    and last_completed_tile[0] == mission["mission_id"]):
                send_reply(radio, sender_mac, "TILE_DONE|%s|%d|%d|%d" % last_completed_tile)
                return last_done
            mission["next_sequence"] = 0
            active_tile_mission = mission
            send_reply(radio, sender_mac, "MISSION_READY|%s|%d" % (
                mission["mission_id"], mission["segment_count"]
            ))
            return last_done

        if text.startswith("SEG|"):
            segment = parse_segment(text)
            mission = active_tile_mission
            key = (segment["mission_id"], segment["sequence"])
            if mission is None or mission["mission_id"] != key[0]:
                if (last_completed_tile is not None
                        and last_completed_tile[0] == key[0]
                        and key[1] == last_completed_tile[3] - 1):
                    send_reply(radio, sender_mac, "DONE|%s|%d" % key)
                    send_reply(radio, sender_mac, "TILE_DONE|%s|%d|%d|%d" % last_completed_tile)
                    return last_done
                send_reply(radio, sender_mac, "ERR|%s|%d|NO_ACTIVE_MISSION" % key)
                return last_done
            if active_path_job is not None:
                if active_path_job["key"] == key:
                    send_reply(radio, sender_mac, "RECEIVED|%s|%d" % key)
                else:
                    send_reply(radio, sender_mac, "ERR|%s|%d|SEGMENT_BUSY" % key)
                return last_done
            if key[1] < mission["next_sequence"]:
                send_reply(radio, sender_mac, "DONE|%s|%d" % key)
                if key[1] == mission["segment_count"] - 1 and last_completed_tile:
                    send_reply(radio, sender_mac, "TILE_DONE|%s|%d|%d|%d" % last_completed_tile)
                return last_done
            if key[1] != mission["next_sequence"] or key[1] >= mission["segment_count"]:
                send_reply(radio, sender_mac, "ERR|%s|%d|SEQUENCE_MISMATCH" % key)
                return last_done
            steps = schedule_segment(segment)
            active_path_job = {
                "key": key, "segment": segment, "steps": steps,
                "step_index": 0, "peer_mac": sender_mac,
            }
            send_reply(radio, sender_mac, "RECEIVED|%s|%d" % key)
            start_path_step(active_path_job)
            return last_done

        if text.startswith("MOVE|"):
            segment = parse_move(text, robot_pose)
        else:
            segment = parse_segment(text)
        key = (segment["mission_id"], segment["sequence"])
        if key == last_done:
            send_reply(radio, sender_mac, "DONE|%s|%d" % key)
            return last_done

        send_reply(radio, sender_mac, "ERR|%s|%d|MISSION_START_REQUIRED" % key)
        return last_done
    except Exception as error:
        if active_path_job is not None:
            robot_pose_valid = False
        stop_motors()
        active_path_job = None
        active_tile_mission = None
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
            radio, sender_mac, raw_payload, None, last_done
        )
    try:
        service_motor_test(radio)
        service_path_execution(radio)
    except Exception as error:
        if active_path_job is not None:
            robot_pose_valid = False
        stop_motors()
        active_motor_test = None
        active_path_job = None
        active_tile_mission = None
        sys.stdout.write("MOTION_SERVICE_ERROR|%s\n" % str(error).replace("|", "/"))
    sleep_ms(10)
