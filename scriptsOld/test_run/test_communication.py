import serial

class test:
    def __init__(self):
        self.serial_communicator = serial.Serial('COM3', 9600)
        self.msg = bytes('1111011','utf-8')

    def send_msg(self):
        self.serial_communicator.write(self.msg)
        print("{}: msg sent".format(self.msg))
            

    def wait_for_msgs(self):
        print("waiting for msg from arduino")
        ack1 = self.serial_communicator.readline()
        print("recieved message is {}".format(ack1))

flag = True
tester = test()

while flag:
    print("in loop")
    tester.wait_for_msgs()
    tester.send_msg()
