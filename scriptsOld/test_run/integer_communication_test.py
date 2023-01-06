import serial
import time

my_communicator = serial.Serial("COM4", 9600)

def main():
    print("in main")
    count = 0
    while True:
        count +=1
        try:
            print("in try")
            my_communicator.write(count)
            print("msg wrote")
            time.sleep(1)
            print("reading incomming message")
            incomming_msg = my_communicator.readline()
            print(f"{incomming_msg} is the incomming message")
        except KeyboardInterrupt:
            print("recieved  keyboard interrput")
            break

if __name__ == "__main__":
    main()