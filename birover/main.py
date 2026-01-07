import serial
from pynput.keyboard import Listener

port = '/dev/cu.HC-05'
baud = 9600

COMMANDS = set(['w', 's', 'a', 'd'])


def get_char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return 'r'


def send_state(ser, state):
    msg = ord(state).to_bytes(1, 'big')
    ser.write(msg)
    print(f"Sent: {state} ({msg})")


try:
    with serial.Serial(port, baud, timeout=1) as ser:
        print("Connected.")
        state = 'r'
        keys = []

        def on_press(key):
            global state, keys
            if (c := get_char(key)) in COMMANDS:
                if c not in keys:
                    keys.append(c)
                state = c

            send_state(ser, state)

        def on_release(key):
            global state, keys
            if (c := get_char(key)) in COMMANDS:
                keys.remove(c)
            state = keys[-1] if keys else 'r'
            send_state(ser, state)

        with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")
