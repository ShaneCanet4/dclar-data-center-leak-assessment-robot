"""L298N-style open-loop dual DC motor output for the robot ESP32.

Verify all GPIOs and the exact L298N module before wiring. Remove the module's
ENA/ENB jumpers before connecting GPIO14/GPIO27 to those inputs.
"""

from machine import Pin, PWM


PWM_FREQUENCY_HZ = 1000
MAX_DUTY_U16 = 49151  # Software ceiling; not a guarantee of safe motor voltage.
MOTOR_A_DIRECTION_INVERTED = False
MOTOR_B_DIRECTION_INVERTED = False

in1 = Pin(26, Pin.OUT, value=0)
in2 = Pin(25, Pin.OUT, value=0)
in3 = Pin(33, Pin.OUT, value=0)
in4 = Pin(32, Pin.OUT, value=0)
ena = PWM(Pin(14), freq=PWM_FREQUENCY_HZ, duty_u16=0)
enb = PWM(Pin(27), freq=PWM_FREQUENCY_HZ, duty_u16=0)


def _set_channel(pwm, direction_a, direction_b, speed, direction):
    speed = int(speed)
    direction = int(direction)
    if direction not in (-1, 0, 1):
        raise ValueError("direction must be -1, 0, or 1")
    speed = max(0, min(speed, MAX_DUTY_U16))
    if direction == 0 or speed == 0:
        pwm.duty_u16(0)
        direction_a.value(0)
        direction_b.value(0)
    elif direction > 0:
        pwm.duty_u16(0)
        direction_a.value(1)
        direction_b.value(0)
        pwm.duty_u16(speed)
    else:
        pwm.duty_u16(0)
        direction_a.value(0)
        direction_b.value(1)
        pwm.duty_u16(speed)


def set_motors(speed_a, dir_a, speed_b, dir_b):
    """Set two independent open-loop motor channels.

    Speed is a duty_u16 value from zero to MAX_DUTY_U16. Direction is -1, 0,
    or 1. Direction zero always disables that channel.
    """
    if MOTOR_A_DIRECTION_INVERTED:
        dir_a = -int(dir_a)
    if MOTOR_B_DIRECTION_INVERTED:
        dir_b = -int(dir_b)
    _set_channel(ena, in1, in2, speed_a, dir_a)
    _set_channel(enb, in3, in4, speed_b, dir_b)


def stop():
    """Disable both motor outputs and set both direction pairs low."""
    ena.duty_u16(0)
    enb.duty_u16(0)
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)


stop()
