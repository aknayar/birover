import pygame
import serial
import sys
import time
import os
from pynput.keyboard import Key, Listener

TICK_RATE = 30  # Hz
DEADZONE = 0.075
TIMEOUT = 0.2  # (s) Resend even if no change
PRECISION_DEFAULT = 1
PRECISION_SPIN = 2
VALUES_DEFAULT = 10
VALUES_SPIN = 3
SHIFT_DEFAULT = 0
SHIFT_SPIN = 5

PORT = '/dev/cu.BiRover'
BAUD = 9600
DELIM = ','
COMMANDS = set(['w', 's', 'a', 'd'])

keys = set()

os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init()
pygame.joystick.init()

clock = pygame.time.Clock()

if pygame.joystick.get_count() == 0:
    print("Please connect a controller.")
    sys.exit(1)

joystick = pygame.joystick.Joystick(0)

running = True

prev_state = (float('inf'), float('inf'), float('inf'), float('inf'))
prev_time = 0


def get_char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return 'r'


def send_message(ser, msg):
    data = bytearray()
    toks = msg.split(DELIM)
    data.append(len(toks))
    data.extend([int(x) for x in toks])

    ser.write(data)


def apply_deadzone(val):
    if abs(val) < DEADZONE:
        return 0
    return val


def serialize_value(val, values, shift):
    # Convert float in the range of -1 to 1 (0.1 granularity) to a 1-byte int
    # 0 -> offset
    return (int(round((min(1, max(-1, val)) + 1) * values))) << shift


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
        listener.daemon = True
        listener.start()

        while running:
            steering, spinning, brake, throttle = 0.0, 0.0, -1.0, -1.0
            if len(keys) > 0:
                if 'esc' in keys:
                    print("Running is now false")
                    running = False
                if 'w' in keys:
                    throttle += 2
                if 's' in keys:
                    brake += 2
                if 'a' in keys:
                    steering -= .5
                if 'd' in keys:
                    steering += .5
                if steering != 0.0 and brake == -1.0 and throttle == -1.0:
                    spinning = 2 * steering
                    steering = 0.0

            else:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    if event.type == pygame.JOYDEVICEREMOVED:
                        print("Controller disconnected.")
                        running = False

                steering = round(apply_deadzone(
                    joystick.get_axis(0)), PRECISION_DEFAULT)
                spinning = round(round(apply_deadzone(
                    joystick.get_axis(2)) * 3) / 3, PRECISION_SPIN)
                brake = round(joystick.get_axis(4), PRECISION_DEFAULT)
                throttle = round(joystick.get_axis(5), PRECISION_DEFAULT)

            steering_ser = serialize_value(
                steering, VALUES_DEFAULT, SHIFT_DEFAULT)
            spinning_ser = serialize_value(
                spinning, VALUES_SPIN, SHIFT_SPIN)
            steering_spinning_ser = steering_ser | spinning_ser

            brake_ser = serialize_value(
                brake, VALUES_DEFAULT, SHIFT_DEFAULT)
            throttle_ser = serialize_value(
                throttle, VALUES_DEFAULT, SHIFT_DEFAULT)

            curr_time = time.time()
            timeout = curr_time - prev_time >= TIMEOUT
            if (steering, spinning, brake, throttle) != prev_state or timeout:
                print(
                    f"\rSteering: {steering}, Spinning: {spinning}, Brake: {brake}, Throttle: {throttle}"
                    f" ->{steering_spinning_ser}, {brake_ser}, {throttle_ser}", end="")

                msg = f"{steering_spinning_ser}{DELIM}{brake_ser}{DELIM}{throttle_ser}"
                send_message(ser, msg)
                prev_time = curr_time

            prev_state = (steering, spinning, brake, throttle)

            if not timeout:
                clock.tick(TICK_RATE)

        pygame.quit()


except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")
