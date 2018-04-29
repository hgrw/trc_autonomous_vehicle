import copy
import numpy as np
import time
import libraries.settings as lsettings
import libraries.gps as gps
import libraries.imu as imu
# import libraries.car as car
import libraries.bearings as bearings
import libraries.pid as lpid

import libraries.geometry as geo

# Lat, Long
waypoints = [[-27.8552616667, 153.151291667],
             [-27.8553216667, 153.151405],
             [-27.8554266667, 153.151566667],
             [-27.8555, 153.151725],
             [-27.8555866667, 153.15189],
             [-27.8557016667, 153.152083333],
             [-27.855865, 153.152198333],
             [-27.85597, 153.152228333],
             [-27.8560216667, 153.152198333],
             [-27.8560783333, 153.15216],
             [-27.8561183333, 153.15206],
             [-27.85611, 153.151958333],
             [-27.856065, 153.151771667],
             [-27.85603, 153.151628333],
             [-27.8560016667, 153.151535],
             [-27.85596, 153.151348333],
             [-27.85584, 153.151086667],
             [-27.8555833333, 153.150935],
             [-27.8554083333, 153.150948333],
             [-27.8553283333, 153.150986667],
             [-27.8552716667, 153.151021667],
             [-27.855235, 153.151121667]]
waypoints.reverse()

pid_cross = lpid.PID(P=2.0, I=0.001, D=0.001)
pid_steer = lpid.PID(P=1.5, I=0.001, D=0.002)

# car.send()
# car.acceleration(25)


def follow_point(point1, point2):
    gps_success = False

    while not gps_success:
        gps_success, coord = gps.getGPS()

    print(coord)

    # Last point
    if point2[0] == -1 and point2[1] == -1:
        point2 = point1

    dist_to_p1 = bearings.coord_dist_meters(
        point1[0], point1[1], coord[0], coord[1])
    dist_to_p2 = bearings.coord_dist_meters(
        point2[0], point2[1], coord[0], coord[1])

    while dist_to_p1 < lsettings.DIST_THRES_METER or dist_to_p2 < lsettings.DIST_THRES_METER:
        # Calculate cross track distance
        cross_track_dist = geo.cross_track_distance(
            point1[0],
            point1[1],
            point2[0],
            point2[1],
            coord[0],
            coord[1]
        )

        # Update PID and get desired error
        desired_heading = pid_cross.update(cross_track_dist)

        # Get current coords long and lat
        # Calculate bearing
        mid_point = geo.midpoint(point1[0], point1[1], point2[0], point2[1])

        actual_heading = bearings.coord_bearing_degrees(mid_point[0], mid_point[1], coord[0], coord[1])

        # Get yaw rate of change
        # TODO: Scale yaw_roc to between 0 and 1
        yaw_roc = imu.get_yaw_roc()
        yaw_roc = yaw_roc / 250

        # Second PID
        # Subtracting angle and moding it
        error_heading = (((actual_heading + desired_heading) % 360) - imu.getCompass()) % 360

        steering_tuned = pid_steer.update(error_heading, delta_term=yaw_roc)
        steering_tuned = steering_tuned + 250

        # TODO: Scale steering_tuned
        print['left', 'right'][steering_tuned < 0], 'steering: ', steering_tuned, ', yaw_roc: ', yaw_roc, 'desired_heading: ', desired_heading

        # Steering
        # car.steer(steering_tuned)
        # car.send()

        # 30 updates per second
        # time.sleep(0.033)
        # TODO: Remove this
        time.sleep(0.01)

        # Check position
        gps_success = False
        while not gps_success:
            gps_success, coords = gps.getGPS()

        dist_to_p1 = bearings.coord_dist_meters(
            point1[0], point1[1], coord[0], coord[1])
        dist_to_p2 = bearings.coord_dist_meters(
            point2[0], point2[1], coord[0], coord[1])


if __name__ == '__main__':
    waypoints_ = copy.deepcopy(waypoints[1:])
    waypoints_.append([-1, -1])

    for point1, point2 in zip(waypoints, waypoints_):
        follow_point(point1, point2)

    # car.stop()
    # car.send()
