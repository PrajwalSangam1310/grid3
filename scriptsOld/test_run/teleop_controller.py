from pynput.keyboard import Key, Listener
import time
import serial

# message byte format 
# first byte to decide command and robot id
# second byte for speed control

class teleop:
    def __init__(self):
        self.rot_speed = 100
        self.fw_speed = 100
        self.my_communicator = serial.Serial('COM4', 9600)
        self.fwdSpeedThreshold = 100
        self.rotSpeedThreshold = 100


    def update_speed(self,key):
        try:
            if key.char == 'i':
                self.fw_speed += 10
                print("increased forward speed")

            elif key.char == 'k':
                self.fw_speed -= 10
                print("decreased forward speed")

            elif key.char == 'l':
                self.rot_speed += 10
                print("increased rot speed")

            elif key.char == 'j':
                self.rot_speed -= 10
                print("decreadsed rot speed to")
            else:
                print(f"Pressed Invalid key for update speed:{key.char}")
            
            if self.rot_speed > self.rotSpeedThreshold:
                self.rot_speed = self.rotSpeedThreshold
            if self.fw_speed > self.fwdSpeedThreshold:
                self.fw_speed = self.fwdSpeedThreshold
            if self.rot_speed < 0:
                self.rot_speed = 0
            if self.fw_speed < 0:
                self.fw_speed = 0
            print(f"rotational_speed is:{self.fw_speed}, forward speed is:{self.rot_speed}")

        except:
            print("exception in update speed")
            return False

    def on_press(self,key):
        # passer = self.my_communicator.readline()
        # print(f"passer string is{passer}")
        self.command_string = ""
        self.update_command(key)
        self.update_speed(key)
        self.pack_commands()
        self.send_msg()
        incomming = self.my_communicator.readline()
        print(f"Incomming string is:{incomming}")
        self.my_communicator.flush()

    def update_command(self,key):
        print('{0} pressed'.format(
            key))
        if key == Key.esc:
            return
        self.command_string = '111110'
        if key.char == 's':
            self.command_string = '101110'
            self.command = 0
            print('backward')
        elif key.char == 'a':
            self.command_string = '111010'
            self.command = 1
            print('anticlock wise')

        elif key.char == 'd':
            self.command_string = '110110'
            self.command= 2
            print('clockwise')

        elif key.char == 'w':
            self.command_string = '011110'
            # self.command_string = '111010'
            self.command = 3
            print('forward')

        elif key.char == 'x':
            self.command_string = '111100'    
            print('dumping')

        else:
            print(f"Command didnt update, character found is:{key.char}")




    def pack_commands(self):
        rot_speed_string = "{0:0>3}".format(self.rot_speed)
        fw_speed_string = "{0:0>3}".format(self.fw_speed)

        self.bot_id  = self.default_botId
        botid_bitstring = b'00'

        if self.command == 0:
            bitCommand = b'00'

        elif self.command == 1:
            bitCommand = b'01'

        elif self.command == 2:
            bitCommand = b'10'

        elif self.command == 3:
            bitCommand = b'11'

        bitCommandString = botid_bitstring + b'0000'+ format(bitCommand, 'b')

        self.msg = self.command_string + fw_speed_string + rot_speed_string
        self.msg = bytes(self.msg,'utf-8')
        print(f"generated command is :{self.msg}")

    def speedbits(self, speed):
        speed

    def on_release(key):
        print('{0} release'.format(
            key))
        # self.command_string = '111100'+str(0)     s     
        print('command ended')
        # msg = bytes(command_string,'utf-8')
        # my_communicator.write(msg)
        if key == Key.esc:
            # Stop listener
            return False

    def send_msg(self):
        self.my_communicator.write(self.msg)

# Collect events until released
my_teleop = teleop()

with Listener(
        on_press=my_teleop.on_press,
        ) as listener:
    listener.join()

