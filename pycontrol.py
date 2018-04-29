import serial

import struct
import serial
import time
from settings import CAR_SERIAL_PORT

car = serial.Serial()
car.port = CAR_SERIAL_PORT
car.baudrate = 9600
car.open()

Stop = 0  # 0:false 1:true
Steer = 0
Acceleration = 0
Brakes = 0


def send():
    global Stop, Steer, Acceleration, Brakes
    car.write(struct.pack("<BBBBB", 0xFF, Stop, Steer, Acceleration, Brakes))
    if Stop:
        Stop = 0
    pass


def stop():
    global Stop
    Stop = 1


def steer(angle=0):
    angle = round(angle)
    angle = min(45, angle)
    angle = max(-45, angle)
    global Steer
    Steer = angle + 90


def acceleration(power=0):  # 0-100
    global Acceleration
    Acceleration = min(max(power, 0), 1)


def brakes(power=0):  # 0-100
    global Brakes
    Brakes = power


if __name__ == "__main__":
    while True:
        brakes(98)
        steer(0)
        send()
        time.sleep(0.1)

STEER_HARD_LEFT = 330
STEER_MIDDLE = 630
STEER_HARD_RIGHT = 1000

ser = None

serialPath = raw_input('enter path to port: ')
ser = serial.Serial(serialPath, 9600)

if __name__=="__main__":
    while True:
        brakes(98)
        steer(0)
        send()
        time.sleep(0.1)

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

        key = raw_input()

        if key == 'a':
            sendData(str(

                chr(0xFF) +
                chr(0) +
                chr(STEER_MIDDLE + 100) +
                chr(0) +
                chr(0)

            ))
            print 'sent: ', str(
                chr(0xFF) +
                chr(0) +
                chr(STEER_MIDDLE + 100) +
                chr(0) +
                chr(0))

        if key == 'd':
            sendData(str(

                chr(0xFF) +
                chr(0) +
                chr(STEER_MIDDLE - 100) +
                chr(0) +
                chr(0)

            ))
            print 'sent: ', str(
                chr(0xFF) +
                chr(0) +
                chr(STEER_MIDDLE - 100) +
                chr(0) +
                chr(0))

main()