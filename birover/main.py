import serial
import time
import numpy as np
import os
from pynput.keyboard import Key, Listener

# fmt: off - Avoid pygame welcome message and window
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
# fmt: on

# Bluetooth constants
PORT = '/dev/cu.BiRover'
BAUD = 9600

# Control constants
TICK_RATE = 30  # Hz
DEADZONE = 0.075
TIMEOUT = 0.250  # (s) Resend even if no change
COMMANDS = set(['w', 's', 'a', 'd'])

# Serialization constants
SEQ_BYTE = (1 << 8) - 1
BITS_DEFAULT = 5
BITS_SPIN = 3
SHIFT_DEFAULT = 0
SHIFT_SPIN = 5
DELIM = ','

# Control state
keys = set()
prev_state = (float('inf'), float('inf'), float('inf'))
prev_time = 0
running = True

pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
joystick = pygame.joystick.Joystick(
    0) if pygame.joystick.get_count() > 0 else None


def get_char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return 'r'


def send_message(ser, msg):
    data = bytearray()
    data.append(SEQ_BYTE)
    data.extend([int(x) for x in msg.split(DELIM)])

    ser.write(data)


def apply_deadzone(val):
    if abs(val) < DEADZONE:
        return 0
    return val


def normalize_value(val):
    return (val + 1) / 2


def serialize_value(val, nbits, shift):
    nvalues = int(2 ** nbits - 1)
    clamped_val = np.clip(val, -1, 1)
    space = np.linspace(-1, 1, nvalues)
    return ((idx := np.argmin(np.abs(space - clamped_val))) << shift, space[idx])


try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print("Connected.")

        def on_press(key):
            if (c := get_char(key)) in COMMANDS and c not in keys:
                keys.add(c)
            if key == Key.esc:
                keys.add('esc')

        def on_release(key):
            if (c := get_char(key)) in COMMANDS:
                keys.discard(c)

        listener = Listener(on_press=on_press, on_release=on_release)
        listener.daemon = True  # Quicker shutdown
        listener.start()

        while running:
            steering_raw, spinning_raw, velocity_raw = 0.0, 0.0, 0.0
            if len(keys) > 0:
                if 'esc' in keys:
                    running = False
                if 'w' in keys:
                    velocity_raw += 1
                if 's' in keys:
                    velocity_raw -= 1
                if 'a' in keys:
                    steering_raw -= .5
                if 'd' in keys:
                    steering_raw += .5
                if steering_raw != 0.0 and velocity_raw == 0.0:
                    spinning_raw = 2 * steering_raw

            elif joystick is not None:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    if event.type == pygame.JOYDEVICEREMOVED:
                        running = False

                steering_raw = apply_deadzone(joystick.get_axis(0))
                spinning_raw = apply_deadzone(joystick.get_axis(2))
                brake_raw = normalize_value(
                    apply_deadzone(joystick.get_axis(4)))
                throttle_raw = normalize_value(
                    apply_deadzone(joystick.get_axis(5)))
                velocity_raw = throttle_raw - brake_raw

            if spinning_raw != 0.0:
                steering_raw = 0.0
                velocity_raw = 0.0

            steering_ser, steering = serialize_value(
                steering_raw, BITS_DEFAULT, SHIFT_DEFAULT)
            spinning_ser, spinning = serialize_value(
                spinning_raw, BITS_SPIN, SHIFT_SPIN)
            steering_spinning_ser = steering_ser | spinning_ser
            velocity_ser, velocity = serialize_value(
                velocity_raw, BITS_DEFAULT, SHIFT_DEFAULT)

            curr_time = time.time()
            timeout = curr_time - prev_time >= TIMEOUT
            if (steering, spinning, velocity) != prev_state or timeout:
                print(
                    f"\rSteering: {steering}, Spinning: {spinning}, "
                    f"Velocity: {velocity}" + ' ' * 30, end='')

                msg = f"{steering_spinning_ser}{DELIM}{velocity_ser}"
                send_message(ser, msg)
                prev_time = curr_time

            prev_state = (steering, spinning, velocity)

            if not timeout:
                clock.tick(TICK_RATE)

        pygame.quit()

except serial.SerialException as e:
    print(f"Unable to connect to BiRover")
