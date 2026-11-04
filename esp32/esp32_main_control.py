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

from board_display import BoardDisplay


# Set these to match the robot ESP32.
WIFI_CHANNEL = 1
ROBOT_MAC = b'\xAA\xBB\xCC\xDD\xEE\xFF'
MAX_RADIO_PAYLOAD = 240
SERIAL_POLL_MS = 10

screen_lines = ["USB serial starting", "ESP-NOW starting"]
display = None


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

while True:
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
