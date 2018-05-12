import time,math
#import libraries.gps as gps
#import libraries.imu as imu
import libraries.car as car
#import libraries.bearings as bearings
#import libraries.pid as lpid
#import libraries.color_blob as cv
from libraries.settings import *



car.acceleration(1)
car.gear('P')






if __name__ == '__main__':
    while True:
        try:
            mode=int(raw_input('Input Steering angle (-60 to 60):'))
            car.steer(mode)
            car.send()
            mode=int(raw_input('Input Brakes (0 - 100):'))
            car.brakes(mode)
            car.send()
            mode=int(raw_input('Input Throttle (0 - 100):'))
            car.acceleration(mode)
            car.send()
            mode=str(raw_input('Input Gear (P R N D):'))
            car.gear(mode)
            car.send()
            print 'got ', mode  
        except ValueError:
            print "Not a number"
        #for j in range(-10, 10):
            #car.steer(-j)
            #car.send()
            #time.sleep(0.2)
	#for i in range(-10, 10):
            #car.steer(i)
            #car.send()
            #time.sleep(0.2)
#        for point in waypoints:
#            print("GOING TO:",point)
#            follow_point(point)
            
        #print("DONE LAP")

    #car.stop()
    #car.send()
    
    
    
    
    
    
#print("Current Compass: ",imu.getCompass()," Coord angle: ",coordsDirection," Steer Error: ",coordsAngleError," Distance to target: ",bearings.coord_dist_meters(coords[0], coords[1], point[0], point[1]))
       
