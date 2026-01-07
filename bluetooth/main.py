import serial
from pynput.keyboard import Key, Listener

port = '/dev/cu.HC-05' 
baud = 9600
state = ord('0')

def on_release(key):
    if key == Key.esc:
        return False

try:
    with serial.Serial(port, baud, timeout=1) as ser:
        print("Connected.")

        def on_press(key):
            global state
            if key == Key.space:
                state ^= 1
                msg = state.to_bytes(1, 'big')
                ser.write(msg)
                print(f"Sent: {state} ({msg})")

        with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
            
except serial.SerialException as e:
    print(f"[Error] {e}")
except KeyboardInterrupt:
    print("Stopping...")
