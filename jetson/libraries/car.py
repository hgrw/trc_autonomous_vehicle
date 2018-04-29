import struct
import serial
import time
from settings import CAR_SERIAL_PORT


car = serial.Serial()
car.port = CAR_SERIAL_PORT
car.baudrate = 9600
car.open()

Stop=0# 0:false 1:true
Steer=0
Acceleration=0
Brakes=0


def send():
    global Stop,Steer,Acceleration,Brakes
    car.write(struct.pack("<BBBBB",0xFF,Stop,Steer,Acceleration,Brakes ))
    if Stop:
        Stop=0
    pass

def stop():
    global Stop
    Stop=1
    


def steer(angle=0):
    angle=round(angle)
    angle=min(45,angle)
    angle=max(-45,angle)
    global Steer
    Steer=angle+90
    
def acceleration(power=0):#0-100
    global Acceleration
    Acceleration=min(max(power,0),1)
    
    
def brakes(power=0):#0-100
    global Brakes
    Brakes=power
    
    
    
if __name__=="__main__":
    while True:
        brakes(98)
        steer(0)
        send()
        time.sleep(0.1)
