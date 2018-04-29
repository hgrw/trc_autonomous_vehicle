import serial
import struct

STEER_HARD_LEFT = 330
STEER_MIDDLE = 630
STEER_HARD_RIGHT = 1000

ser = None

serialPath = raw_input('enter path to port: ')
ser = serial.Serial(serialPath, 9600)

print serialPath

def sendData(data):

    data += "\r\n"
    ser.write(data.encode())

def main():
    global ser
    global STEER_HARD_LEFT
    global STEER_MIDDLE
    global STEER_HARD_RIGHT

    while 1:

        angle = raw_input('steering angle: ')
        print 'got angle: ', angle

        ser.write(struct.pack("<BBBBB", 0xFF, 0, chr(angle + 90), 0, 0))
        print 'wrote: ', angle + 90


main()