import serial
from pynput.keyboard import Listener

PORT = '/dev/cu.BiRover'
BAUD = 9600
COMMANDS = set(['w', 's', 'a', 'd'])

keys = []


def get_char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return 'r'


def send_state(ser):
    state = keys[-1] if keys else 'r'
    msg = ord(state).to_bytes(1, 'big')
    ser.write(msg)
    print(f"Sent: {state} ({msg})")


try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print("Connected.")

        def on_press(key):
            if (c := get_char(key)) in COMMANDS and c not in keys:
                keys.append(c)
            send_state(ser)

        def on_release(key):
            if (c := get_char(key)) in COMMANDS:
                keys.remove(c)
            send_state(ser)

        with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")
