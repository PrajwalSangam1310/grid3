import time
import json
import serial
from pprint import pprint
import random

if __name__ == "__main__":
    print ("Ready...")
    ser  = serial.Serial("COM4", baudrate= 9600, 
           timeout=2.5, 
           parity=serial.PARITY_NONE, 
           bytesize=serial.EIGHTBITS, 
           stopbits=serial.STOPBITS_ONE
        )
    data = {}
    data["operation"] = "sequence"

    data=bytes(json.dumps(data), 'utf-8')
    print (data)
    if ser.isOpen():
        ser.write(data)
        # print(data)
        print("data wrote in the serial port")
        ser.flush()
        while KeyboardInterrupt:
            try:
                # incoming = ser.readline().decode("utf-8")
                ser.write(data)
                ser.flush()
                print("before read line")
                incoming = ser.readline()

                print("incomming done printing incomming")
                print (incoming)
            except Exception as e:
                print (e)
                print("printed exeption")
                break
        ser.close()
    else:
        print ("opening error")