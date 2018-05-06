import subprocess, sys, io, time
import libraries.car as car


if __name__ == '__main__':

    # Run darknet
    process = subprocess.Popen(['./darknet', 'detector', 'demo', 'cfg/coco.data', 'cfg/yolov2-tiny.cfg', 'yolov2-tiny.weights'], stdout=subprocess.PIPE)

    # Set up reader for parsing stdout on separate thread
    reader = io.TextIOWrapper(process.stdout, encoding='utf8')

    # Send one out of every ten stdout reads to arduino
    sendit = 0
    while True:

        # Read from stdout 5 chars at a time
        char = reader.read(5)

        # Look for float denoting relative position of bounding box
        if char == 'rel: ':

            # Send one out of every ten stdout reads to arduino
            if sendit % 10 == 0:
                sendit = 1
                pos = reader.read(9)
                pos = float(pos[1:-1])
                angle = int(pos * 120.0) - 60
                car.steer(angle)
                car.send()
                print('sent angle: ', angle)
        sendit += 1
    output = process.communicate()[0]
    exitCode = process.returncode

    if (exitCode != 0):
        raise ProcessException(command, exitCode, output)
       
