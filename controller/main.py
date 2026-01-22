import pygame
import serial
import sys
import time

TICK_RATE = 60  # Hz
PRECISION = 1
DEADZONE = 0.075
TIMEOUT = 0.2  # (s) Resend even if no change

PORT = '/dev/cu.BiRover'
BAUD = 9600
DELIM = ','

pygame.init()
pygame.joystick.init()

clock = pygame.time.Clock()

if pygame.joystick.get_count() == 0:
    print("Please connect a controller.")
    sys.exit(1)

joystick = pygame.joystick.Joystick(0)

running = True

prev_state = (float('inf'), float('inf'), float('inf'))
prev_time = 0


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


def serialize_value(val):
    # Convert float in the range of -1 to 1 (0.1 granularity) to a 1-byte int
    # 0 -> 16

    return int(min(1, max(-1, val)) * 10) + 16


try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print("Connected.")

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.JOYDEVICEREMOVED:
                    print("Controller disconnected.")
                    running = False

            steering = round(apply_deadzone(joystick.get_axis(0)), PRECISION)
            brake = round(joystick.get_axis(4), PRECISION)
            throttle = round(joystick.get_axis(5), PRECISION)

            steering_ser = serialize_value(steering)
            brake_ser = serialize_value(brake)
            throttle_ser = serialize_value(throttle)

            curr_time = time.time()
            timeout = curr_time - prev_time >= TIMEOUT
            if (steering, brake, throttle) != prev_state or timeout:
                if timeout:
                    print(f"Timeout: {curr_time - prev_time}")

                print(
                    f"\rSteering: {steering}, Brake: {brake}, Throttle: {throttle}"
                    f" ->{steering_ser}, {brake_ser}, {throttle_ser}", end="")

                msg = f"{steering_ser}{DELIM}{brake_ser}{DELIM}{throttle_ser}"
                send_message(ser, msg)
                prev_time = curr_time

            prev_state = (steering, brake, throttle)

            if not timeout:
                clock.tick(TICK_RATE)

except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")

pygame.quit()
