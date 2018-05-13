import struct
import serial
import time
import binascii
#from settings import CAR_SERIAL_PORT


car = serial.Serial()
car.port = '/dev/ttyACM1'
car.baudrate = 9600
car.open()

Stop=0# 0:false 1:true
Steer=0
Acceleration=0
Brakes=50
Gear = 'N'


def send():
    global Steer,Acceleration,Brakes,Gear
    car.write(struct.pack("<BBBBB",0xFF,Steer,Acceleration,Brakes,ord(Gear)))
    #i = struct.pack("<BBBBB",0xFF,Steer,Acceleration,Brakes,ord(Gear))
    #print(binascii.hexlify(i))


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
    Acceleration=min(max(power,0),100)
    
    
def brakes(power=0):#0-100
    global Brakes
    Brakes=min(max(power,0),100)
    
def gear(power):#0-1000
    global Gear
    Gear = power
    
if __name__=="__main__":
    while True:
        brakes(98)
        steer(0)
        send()
        time.sleep(0.1)
