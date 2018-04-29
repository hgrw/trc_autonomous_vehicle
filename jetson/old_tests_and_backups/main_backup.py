import time
import libraries.gps as gps
import libraries.imu as imu
import libraries.car as car
import libraries.bearings as bearings
import libraries.pid as lpid

waypoints = [[-27.855488, 153.150894], [-27.855488, 153.150894]]
car.acceleration(25)


def follow_point(point):
    # Try and get GPS coordinates
    mag = imu.getCompass()

    gps_success = False
    while not gps_success:
        gps_success, coords = gps.getGPS()

    while bearings.coord_dist_meters(coords[0], coords[1], point[0], point[1]) > 10:
        # Get current coords long and lat
        # Calculate bearing
        coordsDirection = bearings.coord_bearing_degrees(coords[0], coords[1],  # Our location
                                                         point[0], point[1])   # waypoint location
        coordsError = coordsDirection - imu.getCompass()
        car.steer(coordsError)
        time.sleep(0.1)


if __name__ == '__main__':
    for point in waypoints:
        follow_point(point)

    car.stop()
    car.send()
