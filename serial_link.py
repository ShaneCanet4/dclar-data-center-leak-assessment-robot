"""Threaded USB serial connection to the MicroPython main-control ESP32."""

import queue
import sys
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


def _missing_pyserial_message():
    return "Install pyserial for this Python: %s -m pip install pyserial" % sys.executable


def available_ports():
    """Return serial-port records with device name and USB descriptor details."""
    if list_ports is None:
        raise RuntimeError(_missing_pyserial_message())
    return list(list_ports.comports())


class SerialLink:
    def __init__(self, port, baudrate=115200):
        if serial is None:
            raise RuntimeError(_missing_pyserial_message())
        self.port = port
        self.events = queue.Queue()
        self._stop = threading.Event()
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                raw_line = self._serial.readline()
                if raw_line:
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if line:
                        self.events.put(line)
            except Exception as error:
                if not self._stop.is_set():
                    self.events.put("SERIAL_ERROR|%s" % error)
                break

    def send_line(self, message):
        message = str(message).strip()
        if not message or "\n" in message or "\r" in message:
            raise ValueError("serial message must be one non-empty line")
        if "\n" not in message:
            message += "\n"
        self._serial.write(message.encode("utf-8"))
        self._serial.flush()

    def close(self):
        self._stop.set()
        if self._serial.is_open:
            self._serial.close()
