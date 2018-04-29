import math
import copy
import numpy as np
import time

import libraries.color_blob as lcolor
import libraries.settings as lsettings
import libraries.gps as gps
import libraries.imu as imu
import libraries.car as car
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

pid_steer = lpid.PID(P=2.0, I=0.001, D=0.001)


def test_pid():
    while True:
        red_target = lcolor.get_largest_blob_x_y(
            lsettings.RED_THRES_LOW, lsettings.RED_THRES_HIGH, radius=10)

        # If there is a target
        if red_target is not None:
            # Error is the left or right from center
            target_x, target_y = red_target
            target_error = target_x - (lsettings.IMAGE_WIDTH / 2)

            turn_strength_pid = pid_steer.update(target_error)

            print('turn_strength_pid: ', turn_strength_pid,
                  ', turn direction: ', 'left' if turn_strength_pid > 0 else 'right')

        time.sleep(0.05)

def steer_scale_value(val):
    """
    Scales between
    """
    return 90 + math.atan(val) * (45 / (math.pi / 2))


def follow_point(point):
    gps_success = False
    while not gps_success:
        gps_success, coord = gps.getGPS()

        if not gps_success:
            print('GPS Error')

    dist_to_p = bearings.coord_dist_meters(
        point[0], point[1], coord[0], coord[1])

    while dist_to_p > lsettings.DIST_THRES_METER:

        heading_want = bearings.coord_bearing_degrees(
            coord[0], coord[1], point[0], point[1])
        heading_actual = imu.getCompass()
        heading_error = heading_want - heading_actual

        if heading_error > 180:
            heading_error = heading_error - 360
        elif heading_error <= -180:
            heading_error = heading_error + 360

        # Color blob
        red_target = lcolor.get_largest_blob_x_y(
            lsettings.RED_THRES_LOW, lsettins.RED_THRES_HIGH, radius=30)

        # If there is a target
        if red_target[0] is not -1:
            # Error is the left or right from center
            target_x, target_y = red_target
            target_error = target_x - (lsettings.IMAGE_WIDTH / 2)

            turn_strength_pid = pid_steer.update(target_error)

        else:
            turn_strength_pid = pid_steer.update(heading_error)

        # Positive is right
        print('turn_strength_pid: ', turn_strength_pid,
              ', turn direction: ', 'left' if turn_strength_pid > 0 else 'right')

        steering_scaled = steer_scale_value(turn_strength_pid)
        car.steer(steering_scaled)
        car.send()

        time.sleep(0.01)

        # Check position
        gps_success = False
        while not gps_success:
            gps_success, coords = gps.getGPS()

            if not gps_success:
                print('GPS Error')

        dist_to_p = bearings.coord_dist_meters(
            point[0], point[1], coord[0], coord[1])


if __name__ == '__main__':
    for p in waypoints:
        follow_point(p)

    # car.stop()
    # car.send()
