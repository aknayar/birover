import serial
from pynput.keyboard import Key, Listener

port = '/dev/cu.HC-05' 
baud = 9600

def send_state(ser, state):
    msg = ord(state).to_bytes(1, 'big')
    ser.write(msg)
    print(f"Sent: {state} ({msg})")

try:
    with serial.Serial(port, baud, timeout=1) as ser:
        print("Connected.")
        state = 'r'

        def on_press(key):
            global state
            if key == Key.up:
                state = 'w'
            elif key == Key.down:
                state = 's'
            elif key == Key.left:
                state = 'a'
            elif key == Key.right:
                state = 'd'
            else:
                state = 'r'
            
            send_state(ser, state)

        def on_release(key):
            global state
            if key == Key.up and state == 'w':
                state = 'r'
            elif key == Key.down and state == 's':
                state = 'r'
            elif key == Key.left and state == 'a':
                state = 'r'
            elif key == Key.right and state == 'd':
                state = 'r'
            send_state(ser, state)

        with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
            
except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")
