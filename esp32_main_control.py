"""USB-to-ESP-NOW text bridge for the main-control ESP32 (MicroPython).

The computer sends one pipe-delimited command per USB-serial line. The board
forwards the text unchanged to the robot and prints received robot messages
back to the computer, also as pipe-delimited lines.

Example computer command: TASK|1|0|0|Zigzag
Example robot reply:      DONE|1|0|0

Keep each radio message at or below 240 encoded bytes. The robot firmware must
use the same Wi-Fi channel and agree on the text command/reply formats.
"""

import network
import espnow
import sys
import uselect
from time import sleep_ms


# TODO: Set these to match the robot ESP32.
WIFI_CHANNEL = 1
ROBOT_MAC = b'\xAA\xBB\xCC\xDD\xEE\xFF'
MAX_RADIO_PAYLOAD = 240
SERIAL_POLL_MS = 10


def write_pc(line):
    """Write one line of plain text to the computer over USB serial."""
    sys.stdout.write(line + "\n")


def mac_to_text(mac):
    """Format a six-byte MAC address as lowercase colon-separated hex."""
    return ":".join("%02x" % byte for byte in mac)


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

write_pc("READY|%d|%s" % (WIFI_CHANNEL, mac_to_text(ROBOT_MAC)))

while True:
    # Relay any waiting robot message to the computer.
    received = radio.recv(0)
    if received:
        sender_mac, raw_payload = received
        try:
            robot_text = raw_payload.decode().strip()
        except Exception:
            robot_text = "<non-text-payload>"
        write_pc("RX|%s|%s" % (mac_to_text(sender_mac), robot_text))

    # Forward one newline-terminated command from the computer, if available.
    if serial_poll.poll(0):
        command = sys.stdin.readline()
        if command:
            command = command.strip()
            if not command:
                continue
            try:
                payload = command.encode()
                if len(payload) > MAX_RADIO_PAYLOAD:
                    raise ValueError("message exceeds %d bytes" % MAX_RADIO_PAYLOAD)
                sent = radio.send(ROBOT_MAC, payload)
                if sent:
                    write_pc("TX|%s" % command)
                else:
                    write_pc("TX_FAIL|%s" % command)
            except Exception as error:
                write_pc("ERROR|%s" % str(error).replace("|", "/"))

    sleep_ms(SERIAL_POLL_MS)
