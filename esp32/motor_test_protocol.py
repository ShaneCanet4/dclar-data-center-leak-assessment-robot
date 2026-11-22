"""Bounded open-loop motor-jog packet format shared by the GUI and robot."""

MAX_MOTOR_TEST_DUTY_U16 = 49151  # 75% of the MicroPython 0..65535 range.
MIN_MOTOR_TEST_DURATION_MS = 100
MAX_MOTOR_TEST_DURATION_MS = 1500


def _validate_token(token):
    token = str(token).strip()
    if not token or len(token) > 20 or any(char in token for char in "|\r\n"):
        raise ValueError("test ID must be 1-20 characters without | or newlines")
    return token


def make_motor_test_message(test_id, direction_a, direction_b, duty_u16, duration_ms):
    """Build a short, bounded jog command for the configured robot ESP32."""
    test_id = _validate_token(test_id)
    direction_a = int(direction_a)
    direction_b = int(direction_b)
    duty_u16 = int(duty_u16)
    duration_ms = int(duration_ms)
    if direction_a not in (-1, 0, 1) or direction_b not in (-1, 0, 1):
        raise ValueError("motor direction must be -1, 0, or 1")
    if direction_a == 0 and direction_b == 0:
        raise ValueError("at least one motor direction must be nonzero")
    if not 1 <= duty_u16 <= MAX_MOTOR_TEST_DUTY_U16:
        raise ValueError("duty must be between 1 and 49151 (75% cap)")
    if not MIN_MOTOR_TEST_DURATION_MS <= duration_ms <= MAX_MOTOR_TEST_DURATION_MS:
        raise ValueError("duration must be between 100 and 1500 ms")
    return "MOTOR_TEST|%s|%d|%d|%d|%d" % (
        test_id, direction_a, direction_b, duty_u16, duration_ms
    )


def parse_motor_test_message(message):
    """Parse and validate MOTOR_TEST|id|dirA|dirB|duty|duration_ms."""
    fields = str(message).strip().split("|")
    if len(fields) != 6 or fields[0] != "MOTOR_TEST":
        raise ValueError("BAD_MOTOR_TEST_FORMAT")
    try:
        values = _parse_motor_test_values(fields[2], fields[3], fields[4], fields[5])
        values["test_id"] = _validate_token(fields[1])
        return values
    except (TypeError, ValueError):
        raise ValueError("BAD_MOTOR_TEST_VALUE")


def _parse_motor_test_values(direction_a, direction_b, duty_u16, duration_ms):
    direction_a = int(direction_a)
    direction_b = int(direction_b)
    duty_u16 = int(duty_u16)
    duration_ms = int(duration_ms)
    if direction_a not in (-1, 0, 1) or direction_b not in (-1, 0, 1):
        raise ValueError("bad direction")
    if direction_a == 0 and direction_b == 0:
        raise ValueError("both motors stopped")
    if not 1 <= duty_u16 <= MAX_MOTOR_TEST_DUTY_U16:
        raise ValueError("bad duty")
    if not MIN_MOTOR_TEST_DURATION_MS <= duration_ms <= MAX_MOTOR_TEST_DURATION_MS:
        raise ValueError("bad duration")
    return {
        "direction_a": direction_a,
        "direction_b": direction_b,
        "duty_u16": duty_u16,
        "duration_ms": duration_ms,
    }


def make_motor_stop_message(test_id):
    return "MOTOR_STOP|%s" % _validate_token(test_id)


def parse_motor_stop_message(message):
    fields = str(message).strip().split("|")
    if len(fields) != 2 or fields[0] != "MOTOR_STOP":
        raise ValueError("BAD_MOTOR_STOP_FORMAT")
    return _validate_token(fields[1])


def make_robot_radio_ping_message(token):
    return "RADIO_PING|%s" % _validate_token(token)
