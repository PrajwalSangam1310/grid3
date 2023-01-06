from pynput import keyboard
count = 0

class dummy:
    def __init__(self):
        self.count = 0
    def on_press(self,key):
        if self.count >= 10:
            return False
        try:
            char = key.char
            if char == 'w':
                self.count += 1
            if char == 's':
                self.count -= 1
        except:
            return False

test = dummy()
key_listener = keyboard.Listener(on_press = test.on_press)
key_listener.start()

while test.count < 10:
    try:
        print(f"present count is:{test.count}")
    except:
        break


key_listener.join()
