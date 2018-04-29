import FaBo9Axis_MPU9250
import math
from settings import ROLLING_AMOUNT
imu = FaBo9Axis_MPU9250.MPU9250()

direction = imu.readMagnet()
direction = 180-math.degrees(math.atan2( direction['y'],direction['x']))

rolling=[0,0]
def getCompass():

    global direction
    mag = imu.readMagnet()
    if mag['x'] == 0.0 and mag['y'] == 0.0:
        return direction
    rolling[0]=(rolling[0]*ROLLING_AMOUNT+mag['x'])/(ROLLING_AMOUNT+1)
    rolling[1]=(rolling[1]*ROLLING_AMOUNT+mag['y'])/(ROLLING_AMOUNT+1)
    
    val = math.atan2(rolling[1],rolling[0])

    direction = 180-math.degrees(val)
    return direction#+11




def get_yaw_roc():
    """
    Get yaw rate of change
    """
    gyro = imu.readGyro()
    return gyro['z']
