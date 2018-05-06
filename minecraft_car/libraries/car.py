import struct
import serial
import time
#from settings import CAR_SERIAL_PORT


car = serial.Serial()
car.port = '/dev/ttyACM0'
car.baudrate = 9600
car.open()

Stop=0# 0:false 1:true
Steer=0
Acceleration=0
Brakes=0


def send():
    global Stop,Steer,Acceleration,Brakes
    car.write(struct.pack("<BBBBB",0xFF,Stop,Steer,Acceleration,Brakes))
    if Stop:
        Stop=0
    pass

def stop():
    global Stop
    Stop=1
    

# Normalise steering angle to between -60, 60 degrees, add 60 so that it's positive
def steer(angle=0):
    angle=round(angle)
    angle=min(60,angle)
    angle=max(-60,angle)
    global Steer
    Steer=angle+60
    
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
