"""USB-to-ESP-NOW text bridge for the main-control ESP32 (MicroPython).

The desktop sends one pipe-delimited command per USB serial line. This board
answers PING locally, forwards other messages unchanged to the robot, and
relays robot replies to both USB serial and the built-in TFT display.

Examples:
    Computer -> board: PING|abc123
    Board -> computer: PONG|abc123
    Computer -> board: MISSION_START|H0001|0|0|Zigzag|18
    Computer -> board: SEG|H0001|0|...
    Robot -> board:    DONE|H0001|0|x|y|heading
    Robot -> board:    TILE_DONE|H0001|0|0|18

Use the same Wi-Fi channel on both ESP32s. Keep radio messages at or below
240 encoded bytes. The bridge forwards tile manifests, individual segments,
and the robot's confirmations without owning the mission state. See README.md
for the text protocol and board setup.
"""

import network
import espnow
import math
import sys
import uselect
from time import sleep_ms
from time import ticks_ms, ticks_diff

from board_display import BoardDisplay


# Set these to match the robot ESP32.
WIFI_CHANNEL = 1
ROBOT_MAC = b'\xAA\xBB\xCC\xDD\xEE\xFF'
MAX_RADIO_PAYLOAD = 240
SERIAL_POLL_MS = 10
LOCAL_PATH_TEST_DUTY_U16 = 19660  # 30%; short signal test, not calibrated travel.
LOCAL_TEST_MS_PER_DEGREE = 2.5
LOCAL_TEST_MS_PER_FOOT = 180.0
LOCAL_TEST_MIN_STEP_MS = 60
LOCAL_TEST_MAX_STEP_MS = 500
MAX_LOCAL_TILE_SEGMENTS = 300

screen_lines = ["USB serial starting", "ESP-NOW starting"]
display = None
fake_robot_mode = False
local_motor_driver = None
active_local_motor_test = None
active_local_tile_mission = None
active_local_path_job = None
local_path_test_active = False


def write_pc(line):
    """Write one line of plain text to the computer over USB serial."""
    sys.stdout.write(line + "\n")


def mac_to_text(mac):
    """Format a six-byte MAC address as lowercase colon-separated hex."""
    return ":".join("%02x" % byte for byte in mac)


def show_status(line):
    """Keep the latest status visible on the built-in TFT, if it initialized."""
    global screen_lines
    # Wrap long task f-strings so coordinates and instruction fields remain
    # readable across the TFT's 30-character text width.
    text = str(line)
    wrapped = [text[index:index + 30] for index in range(0, len(text), 30)] or [""]
    if local_path_test_active:
        screen_lines = ["LOCAL PATH TEST"] + wrapped[:2]
    elif fake_robot_mode:
        screen_lines = ["FAKE ROBOT ON"] + wrapped[:2]
    else:
        screen_lines = wrapped + screen_lines[:max(0, 7 - len(wrapped))]
    if display is not None:
        try:
            display.show(screen_lines)
        except Exception as error:
            write_pc("DISPLAY_ERROR|%s" % str(error).replace("|", "/"))


# Initialize the built-in ST7789 screen from board_display.py.
try:
    display = BoardDisplay()
    display.show(screen_lines)
except Exception as error:
    write_pc("DISPLAY_ERROR|%s" % str(error).replace("|", "/"))

# Enable STA mode and select the channel shared with the robot ESP32.
sta = network.WLAN(network.STA_IF)
sta.active(True)
try:
    sta.disconnect()
except Exception:
    pass
sta.config(channel=WIFI_CHANNEL)

# Activate ESP-NOW and register the robot as a peer.
radio = espnow.ESPNow()
radio.active(True)
radio.add_peer(ROBOT_MAC)

# Poll USB serial without blocking ESP-NOW message reception.
serial_poll = uselect.poll()
serial_poll.register(sys.stdin, uselect.POLLIN)

ready_line = "READY|%d|%s" % (WIFI_CHANNEL, mac_to_text(ROBOT_MAC))
write_pc(ready_line)
show_status("Ready ch %d" % WIFI_CHANNEL)
show_status("Robot " + mac_to_text(ROBOT_MAC))


def stop_local_motor_test():
    global active_local_motor_test
    if local_motor_driver is not None:
        local_motor_driver.stop()
    active_local_motor_test = None


def service_local_motor_test():
    global active_local_motor_test
    if active_local_motor_test is None:
        return
    elapsed = ticks_diff(ticks_ms(), active_local_motor_test["started_ms"])
    if elapsed >= active_local_motor_test["duration_ms"]:
        token = active_local_motor_test["test_id"]
        stop_local_motor_test()
        write_pc("RX|LOCAL|MOTOR_TEST_DONE|%s|%d" % (token, elapsed))
        show_status("Jog done " + token)


def handle_local_motor_command(command):
    """Run the existing short motor-jog protocol directly on this board."""
    global active_local_motor_test
    fields = command.split("|")
    if fields[0] == "MOTOR_STOP":
        token = fields[1] if len(fields) == 2 else "STOP"
        stop_local_motor_test()
        write_pc("RX|LOCAL|MOTOR_TEST_STOPPED|%s|0" % token)
        show_status("Motors stopped")
        return
    token = "?"
    try:
        from motor_test_protocol import parse_motor_test_message
        test = parse_motor_test_message(command)
        if active_local_tile_mission is not None:
            raise ValueError("LOCAL_PATH_BUSY")
        token = test["test_id"]
        if active_local_motor_test is not None:
            raise ValueError("MOTOR_TEST_BUSY")
        if local_motor_driver is None:
            raise ValueError("MOTOR_DRIVER_UNAVAILABLE")
        if test["duty_u16"] > local_motor_driver.MAX_DUTY_U16:
            raise ValueError("DUTY_ABOVE_DRIVER_LIMIT")
        local_motor_driver.stop()
        local_motor_driver.set_motors(
            test["duty_u16"], test["direction_a"],
            test["duty_u16"], test["direction_b"],
        )
        active_local_motor_test = {
            "test_id": token,
            "started_ms": ticks_ms(),
            "duration_ms": test["duration_ms"],
        }
        write_pc("RX|LOCAL|MOTOR_TEST_STARTED|%s|%d" % (token, test["duration_ms"]))
        show_status("Local jog " + token)
    except Exception as error:
        stop_local_motor_test()
        reason = str(error).replace("|", "/").replace(" ", "_")
        write_pc("RX|LOCAL|MOTOR_TEST_ERR|%s|%s" % (token, reason))
        show_status("Motor error " + reason)


def stop_local_path_test(report=True):
    global active_local_tile_mission, active_local_path_job, local_path_test_active
    mission = active_local_tile_mission
    if local_motor_driver is not None:
        local_motor_driver.stop()
    active_local_tile_mission = None
    active_local_path_job = None
    local_path_test_active = False
    if report and mission is not None:
        write_pc("RX|LOCAL|MISSION_STOPPED|%s" % mission["mission_id"])
    show_status("Local test motors stopped")


def _local_test_duration(raw_ms):
    return max(LOCAL_TEST_MIN_STEP_MS, min(LOCAL_TEST_MAX_STEP_MS, int(raw_ms)))


def _start_local_path_step():
    step = active_local_path_job["steps"][active_local_path_job["step_index"]]
    local_motor_driver.set_motors(
        LOCAL_PATH_TEST_DUTY_U16, step[1],
        LOCAL_PATH_TEST_DUTY_U16, step[2],
    )
    active_local_path_job["started_ms"] = ticks_ms()
    active_local_path_job["duration_ms"] = step[0]


def handle_local_path_command(command):
    """Locally emulate a robot tile transaction using short, bounded motor pulses."""
    global active_local_tile_mission, active_local_path_job, local_path_test_active
    fields = command.split("|")
    if fields[0] == "MISSION_STOP":
        mission_id = fields[1] if len(fields) == 2 else "?"
        if (active_local_tile_mission is None
                or active_local_tile_mission["mission_id"] != mission_id):
            write_pc("RX|LOCAL|ERR|%s|0|MISSION_ID_MISMATCH" % mission_id)
            return
        stop_local_path_test()
        return

    mission_id = fields[1] if len(fields) > 1 else "?"
    sequence = fields[2] if len(fields) > 2 else "0"
    try:
        if local_motor_driver is None:
            raise ValueError("MOTOR_DRIVER_UNAVAILABLE")
        if active_local_motor_test is not None:
            raise ValueError("MOTOR_TEST_BUSY")
        if fields[0] == "MISSION_START":
            if len(fields) != 6:
                raise ValueError("EXPECTED_MISSION_START_FORMAT")
            row, column, count = int(fields[2]), int(fields[3]), int(fields[5])
            pattern = fields[4]
            if row < 0 or column < 0 or count < 1 or count > MAX_LOCAL_TILE_SEGMENTS:
                raise ValueError("BAD_TILE_MANIFEST")
            if pattern not in ("Zigzag", "Circle_spiral"):
                raise ValueError("UNSUPPORTED_PATTERN")
            if LOCAL_PATH_TEST_DUTY_U16 > local_motor_driver.MAX_DUTY_U16:
                raise ValueError("LOCAL_TEST_DUTY_ABOVE_DRIVER_LIMIT")
            if active_local_tile_mission is not None:
                if (active_local_tile_mission["mission_id"] == mission_id
                        and active_local_tile_mission["segment_count"] == count):
                    write_pc("RX|LOCAL|MISSION_READY|%s|%d" % (mission_id, count))
                    return
                raise ValueError("LOCAL_PATH_BUSY")
            active_local_tile_mission = {
                "mission_id": mission_id, "row": row, "column": column,
                "pattern": pattern, "segment_count": count, "next_sequence": 0,
            }
            local_path_test_active = True
            local_motor_driver.stop()
            show_status("Manifest " + mission_id)
            write_pc("RX|LOCAL|MISSION_READY|%s|%d" % (mission_id, count))
            return

        if fields[0] != "SEG" or len(fields) != 8:
            raise ValueError("EXPECTED_SEGMENT_FORMAT")
        mission = active_local_tile_mission
        if mission is None or mission["mission_id"] != mission_id:
            raise ValueError("NO_ACTIVE_LOCAL_PATH")
        sequence_number = int(sequence)
        if active_local_path_job is not None:
            if active_local_path_job["sequence"] == sequence_number:
                write_pc("RX|LOCAL|RECEIVED|%s|%d" % (mission_id, sequence_number))
                return
            raise ValueError("LOCAL_SEGMENT_BUSY")
        if sequence_number != mission["next_sequence"] or sequence_number >= mission["segment_count"]:
            raise ValueError("SEQUENCE_MISMATCH")
        pivot = float(fields[3])
        distance_ft = float(fields[4])
        x_ft, y_ft, heading = float(fields[5]), float(fields[6]), float(fields[7])
        values = (pivot, distance_ft, x_ft, y_ft, heading)
        for value in values:
            if value != value or abs(value) == float("inf"):
                raise ValueError("NON_FINITE_SEGMENT")
        if abs(pivot) > 180 or distance_ft < 0 or distance_ft > 2.5:
            raise ValueError("SEGMENT_OUT_OF_RANGE")
        steps = []
        if abs(pivot) > 0.01:
            turn = 1 if pivot > 0 else -1
            steps.append((_local_test_duration(abs(pivot) * LOCAL_TEST_MS_PER_DEGREE), turn, -turn))
        if distance_ft > 0:
            drive_ms = _local_test_duration(distance_ft * LOCAL_TEST_MS_PER_FOOT)
            steps.append((drive_ms, 1, 1))
        if not steps:
            raise ValueError("EMPTY_SEGMENT")
        active_local_path_job = {
            "sequence": sequence_number, "steps": steps, "step_index": 0,
            "x_ft": x_ft, "y_ft": y_ft, "heading": heading,
        }
        write_pc("RX|LOCAL|RECEIVED|%s|%d" % (mission_id, sequence_number))
        show_status("Pulse %d/%d" % (sequence_number + 1, mission["segment_count"]))
        _start_local_path_step()
    except Exception as error:
        if local_motor_driver is not None:
            local_motor_driver.stop()
        active_local_path_job = None
        if fields[0] == "MISSION_START":
            active_local_tile_mission = None
            local_path_test_active = False
        reason = str(error).replace("|", "/").replace(" ", "_")
        write_pc("RX|LOCAL|ERR|%s|%s|%s" % (mission_id, sequence, reason))
        show_status("Path test error " + reason)


def service_local_path_test():
    global active_local_tile_mission, active_local_path_job, local_path_test_active
    job = active_local_path_job
    if job is None:
        return
    elapsed = ticks_diff(ticks_ms(), job["started_ms"])
    if elapsed < job["duration_ms"]:
        return
    local_motor_driver.stop()
    job["step_index"] += 1
    if job["step_index"] < len(job["steps"]):
        _start_local_path_step()
        return
    mission = active_local_tile_mission
    mission_id = mission["mission_id"]
    sequence = job["sequence"]
    mission["next_sequence"] += 1
    active_local_path_job = None
    write_pc("RX|LOCAL|DONE|%s|%d|%.4f|%.4f|%.2f" % (
        mission_id, sequence, job["x_ft"], job["y_ft"], job["heading"]
    ))
    if mission["next_sequence"] >= mission["segment_count"]:
        if (abs(job["x_ft"] - (mission["column"] + 0.5)) > 0.03
                or abs(job["y_ft"] - (mission["row"] + 0.5)) > 0.03):
            write_pc("RX|LOCAL|ERR|%s|%d|TILE_ENDPOINT_MISMATCH" % (mission_id, sequence))
        else:
            write_pc("RX|LOCAL|TILE_DONE|%s|%d|%d|%d" % (
                mission_id, mission["row"], mission["column"], mission["segment_count"]
            ))
            show_status("Tile done " + mission_id)
        active_local_tile_mission = None
        local_path_test_active = False


def set_fake_robot_mode(enabled):
    global fake_robot_mode, local_motor_driver
    if enabled:
        try:
            import main_control_motor_driver
            main_control_motor_driver.stop()
            local_motor_driver = main_control_motor_driver
        except Exception as error:
            local_motor_driver = None
            write_pc("ERROR|MOTOR_DRIVER|%s" % str(error).replace("|", "/"))
            show_status("Fake mode error")
            return
        fake_robot_mode = True
        show_status("FAKE ROBOT ON")
        write_pc("MODE|FAKE_ROBOT|ON")
    else:
        stop_local_motor_test()
        stop_local_path_test()
        fake_robot_mode = False
        show_status("Radio bridge mode")
        write_pc("MODE|FAKE_ROBOT|OFF")

while True:
    service_local_motor_test()
    try:
        service_local_path_test()
    except Exception as error:
        mission_id = (
            active_local_tile_mission["mission_id"]
            if active_local_tile_mission is not None else "?"
        )
        sequence = (
            active_local_path_job["sequence"]
            if active_local_path_job is not None else "?"
        )
        stop_local_path_test(report=False)
        reason = str(error).replace("|", "/").replace(" ", "_")
        write_pc("RX|LOCAL|ERR|%s|%s|%s" % (mission_id, sequence, reason))
    # Relay any waiting robot message to the computer and display.
    received = radio.recv(0)
    # MicroPython 1.28 returns (None, None), a truthy tuple, when recv(0)
    # finds no packet. Check both fields so the idle loop does not treat that
    # timeout sentinel as a real robot message.
    if received and received[0] is not None and received[1] is not None:
        sender_mac, raw_payload = received
        try:
            robot_text = raw_payload.decode().strip()
        except Exception:
            robot_text = "<non-text-payload>"
        write_pc("RX|%s|%s" % (mac_to_text(sender_mac), robot_text))
        show_status("Robot: " + robot_text)

    # Forward one newline-terminated command from the computer, if available.
    if serial_poll.poll(0):
        command = sys.stdin.readline()
        if command:
            command = command.strip()
            if not command:
                continue

            # PING checks the computer-to-board USB path without needing the
            # robot firmware. The display shows both the request and response.
            fields = command.split("|")
            if len(fields) == 2 and fields[0] == "PING" and fields[1]:
                response = "PONG|" + fields[1]
                show_status("USB: " + command)
                write_pc(response)
                show_status("USB: " + response)
                continue

            if len(fields) == 2 and fields[0] == "FAKE_ROBOT" and fields[1] in ("ON", "OFF"):
                set_fake_robot_mode(fields[1] == "ON")
                continue

            if fake_robot_mode and (fields[0] == "MOTOR_TEST" or fields[0] == "MOTOR_STOP"):
                handle_local_motor_command(command)
                continue

            if fake_robot_mode and fields[0] in (
                    "MISSION_START", "SEG", "MISSION_STOP"):
                handle_local_path_command(command)
                continue

            try:
                payload = command.encode()
                if len(payload) > MAX_RADIO_PAYLOAD:
                    raise ValueError("message exceeds %d bytes" % MAX_RADIO_PAYLOAD)
                sent = radio.send(ROBOT_MAC, payload)
                if sent:
                    write_pc("TX|%s" % command)
                    show_status("Sent: " + command)
                else:
                    write_pc("TX_FAIL|%s" % command)
                    show_status("Send failed")
            except Exception as error:
                message = str(error).replace("|", "/")
                write_pc("ERROR|%s" % message)
                show_status("Error: " + message)

    sleep_ms(SERIAL_POLL_MS)
