import serial
import pynmea2
import sys
import math

from settings import GPS_SERIAL_PORT

# GPS CONFIG MACROS
PMTK_SET_NMEA_BAUDRATE = '$PMTK251,9600*17'
PMTK_SET_NMEA_UPDATE_5HZ = "$PMTK220,200*2C"
PMTK_SET_NMEA_OUTPUT_RMCONLY = '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29'
PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
PMTK_SET_NMEA_OUTPUT_GGAONLY = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"


# GPS CONFIG ROUTINE
serial_gps = serial.Serial()
serial_gps.port = GPS_SERIAL_PORT
serial_gps.baudrate = 9600
serial_gps.open()
serial_gps.write(PMTK_SET_NMEA_BAUDRATE + '\r\n')
serial_gps.write(PMTK_SET_NMEA_OUTPUT_GGAONLY + '\r\n')
serial_gps.write(PMTK_SET_NMEA_UPDATE_5HZ + '\r\n')


def getGPS(NUM_SATS_NEEDED=4):
    msg = ""
    try:
        line = serial_gps.readline()
        try:  # try statement so that GGAONLY doesn't catch the initial line and crash
            msg = pynmea2.parse(line, check=True)
        except:
            return False, msg
        try:
            if int(msg.num_sats) >= NUM_SATS_NEEDED:
                return True, [msg.latitude, msg.longitude]
            else:
                return False, msg
        except:
            return False, msg
    except:
        return False, msg
