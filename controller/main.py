import pygame
import serial
import sys
import time

TICK_RATE = 15  # Hz
PRECISION = 2

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


def send_message(ser, msg):
    data = bytearray()
    toks = msg.split(DELIM)
    data.append(len(toks))
    data.extend([int(x) for x in toks])

    ser.write(data)


def serialize_value(val):
    # Convert float in the range of -1 to 1 (0.1 granularity) to a 1-byte int
    # 0 -> 128

    return int(min(1, max(-1, val)) * 100) + 128


try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print("Connected.")

        # ser.write(int(0).to_bytes(1, 'big'))
        # print("Sent zero")

        # while int.from_bytes(ser.read(1), 'big') != 1:
        #     continue
        # print("Got one")

        # time.sleep(.25)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.JOYDEVICEREMOVED:
                    print("Controller disconnected.")
                    running = False

            steering = round(joystick.get_axis(0), PRECISION)
            brake = round(joystick.get_axis(4), PRECISION)
            throttle = round(joystick.get_axis(5), PRECISION)

            steering_ser = serialize_value(steering)
            brake_ser = serialize_value(brake)
            throttle_ser = serialize_value(throttle)

            print(
                f"\rSteering: {steering}, Brake: {brake}, Throttle: {throttle} -> {steering_ser}, {brake_ser}, {throttle_ser}", end="")

            msg = f"{steering_ser}{DELIM}{brake_ser}{DELIM}{throttle_ser}"
            send_message(ser, msg)

            clock.tick(TICK_RATE)

except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")

pygame.quit()
