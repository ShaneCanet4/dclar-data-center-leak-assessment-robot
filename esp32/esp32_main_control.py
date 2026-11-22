"""USB-to-ESP-NOW text bridge for the main-control ESP32 (MicroPython).

The desktop sends one pipe-delimited command per USB serial line. This board
answers PING locally, forwards other messages unchanged to the robot, and
relays robot replies to both USB serial and the built-in TFT display.

Examples:
    Computer -> board: PING|abc123
    Board -> computer: PONG|abc123
    Computer -> board: TASK|1|0|0|Zigzag
    Robot -> board:    DONE|1|0|0

Use the same Wi-Fi channel on both ESP32s. Keep radio messages at or below
240 encoded bytes. See README.md for the text protocol and board setup.
"""

import network
import espnow
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

screen_lines = ["USB serial starting", "ESP-NOW starting"]
display = None
fake_robot_mode = False
local_motor_driver = None
active_local_motor_test = None


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
    if fake_robot_mode:
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
        fake_robot_mode = False
        show_status("Radio bridge mode")
        write_pc("MODE|FAKE_ROBOT|OFF")

while True:
    service_local_motor_test()
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
