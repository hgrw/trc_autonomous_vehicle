import libraries.gps as gps
import libraries.imu as imu
import libraries.car as car
import math

car.send()

x = gps.getGPS()

while True:
    # print(math.degrees(imu.getCompass()))
    if x != gps.getGPS():
        x = gps.getGPS()
        # print(x)
        print(x.latitude, x.longitude)
