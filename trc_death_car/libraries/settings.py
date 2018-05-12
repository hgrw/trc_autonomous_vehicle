# Serial port settings
CAR_SERIAL_PORT = '/dev/ttyACM1'
GPS_SERIAL_PORT = '/dev/ttyUSB0'

# How far do we have to be from a point
# before we consider it to be 'close enough'
DIST_THRES_METER = 10


#radius of trak used for GPS kill switch
TRACK_RADIUS=1200

# Compass rolling averag0
ROLLING_AMOUNT = 1

# Image properties
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080

# Color thresholding
RADIUS = 10
RED_THRES_LOW = (160, 120, 20)
RED_THRES_HIGH = (185, 220, 200)
