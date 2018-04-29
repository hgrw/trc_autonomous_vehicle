import time
import libraries.gps as gps

with open("gps_coords.txt", "a") as myfile:
    gps_success = False

    for i in range(10):
        gps_success, coord = gps.getGPS()
        time.sleep(0.05)

        if gps_success:
            break

    if gps_success:
        myfile.write(str(coord[0]) + ', ' + str(coord[1]) + '\n')
        print "Wrote file"

    else:
        print "Error GPS"
