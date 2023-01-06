from pynput import keyboard
import time

my_keyboard = keyboard.Controller()

while True:
    my_keyboard.press(keyboard.Key.esc)
    print("loop running")
    time.sleep(0.2)