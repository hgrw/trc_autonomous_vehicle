import time
import libraries.gps as gps
import libraries.imu as imu
while True:
    print(gps.getGPS())#imu.getCompass())
    time.sleep(0.05)
