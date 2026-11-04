"""Hardware check for the computer <-> main-control ESP32 USB connection.

Run with ``python test_esp32_communication.py`` and choose the ESP32 COM port,
or pass it directly, for example ``python test_esp32_communication.py --port COM5``.
The main-control ESP32 must be running esp32/esp32_main_control.py.
"""

import argparse
import sys
import time

import serial

from serial_link import available_ports
from serial_protocol import make_ping_message


def choose_port(explicit_port=None):
    if explicit_port:
        return explicit_port
    port_records = available_ports()
    if not port_records:
        raise RuntimeError("No serial ports found. Connect the ESP32 over USB-C.")
    print("Available serial ports:")
    for index, port_info in enumerate(port_records, 1):
        description = port_info.description or "USB serial device"
        print("  %d. %s — %s" % (index, port_info.device, description))
    selection = input("Choose a port number or enter a COM name: ").strip()
    if selection.isdigit() and 1 <= int(selection) <= len(port_records):
        return port_records[int(selection) - 1].device
    return selection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial port, such as COM5")
    parser.add_argument("--timeout", type=float, default=8.0, help="seconds to wait for PONG")
    args = parser.parse_args()

    port = choose_port(args.port)
    token = "PC%04d" % (int(time.time()) % 10000)
    expected = "PONG|" + token
    print("Opening %s and sending %s" % (port, make_ping_message(token)))

    with serial.Serial(port, baudrate=115200, timeout=0.2, write_timeout=2) as connection:
        # USB CDC may reset the board when the port opens. Wait briefly, then
        # clear old startup text so the response check is unambiguous.
        time.sleep(1.5)
        connection.reset_input_buffer()
        connection.write((make_ping_message(token) + "\n").encode("utf-8"))
        connection.flush()
        deadline = time.monotonic() + args.timeout
        seen = []
        while time.monotonic() < deadline:
            raw_line = connection.readline()
            if not raw_line:
                continue
            line = raw_line.decode("utf-8", errors="replace").strip()
            seen.append(line)
            print("ESP32: " + line)
            if line == expected:
                print("PASS: USB serial PING/PONG round trip succeeded.")
                print("Check the board TFT; it should show the PING and PONG lines.")
                return 0

    print("FAIL: no matching %s response within %.1f seconds." % (expected, args.timeout))
    if seen:
        print("Received lines: " + repr(seen))
    else:
        print("No serial lines received; check the selected port and firmware.")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, serial.SerialException, ValueError) as error:
        print("ERROR: %s" % error, file=sys.stderr)
        sys.exit(2)
