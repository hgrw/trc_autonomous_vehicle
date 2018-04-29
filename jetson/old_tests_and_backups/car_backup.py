import struct
import serial

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
    global Stop,Steer,Acceleration,Gear,Brakes 
    car.write(':'.join([str(i) for i in [Stop,Steer,Acceleration,Brakes]]))
    if Stop:
        Stop=0
    pass

def stop():
    global Stop
    Stop=1
    


def steer(angle=0):
    angle=round(angle)
    angle=min(90,angle)
    angle=max(-90,angle)
    global Steer
    Steer=angle+90
    
def acceleration(power=0):#0-100
    global Acceleration
    Acceleration=power
def brakes(power=0):#0-100
    global Brakes
    Brakes=power
if __name__=="__main__":
    for i in range(1000):
        brakes(98)
        steer(100)
        send()
