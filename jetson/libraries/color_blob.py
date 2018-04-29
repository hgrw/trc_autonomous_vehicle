import cv2
import imutils
import numpy as np

from libraries.settings import RED_THRES_LOW, RED_THRES_HIGH, IMAGE_WIDTH, IMAGE_HEIGHT

IMG_DEBUG = None
CAMERA = cv2.VideoCapture(1)
CAMERA.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
CAMERA.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
def get_largest_blob_x_y(low_thres, high_thres, radius=10):
    """
    Get largest blob between low and high thres, radius is arbitrary

    :params low_thres: tuple
    :params high_thres: tuple
    :params radius: int
    :return
    """
    (grabbed, frame) = CAMERA.read()

    if not grabbed:
        return [-1,-1]

    frame_resized = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
    frame_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(frame_hsv, low_thres, high_thres)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), r) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        except:
            return [-1,-1]

        if r >= radius:
            return center

    return [-1,-1]


def on_mouse(event, x, y, flag, param):
    global IMG_DEBUG, IMG_DEBUG_HSV

    if event == cv2.EVENT_LBUTTONDBLCLK:
        # Circle to indicate hsv location, and update frame
        cv2.circle(IMG_DEBUG, (x, y), 3, (0, 0, 255))
        cv2.imshow('hsv_extractor', IMG_DEBUG)

        # Print values
        values = IMG_DEBUG_HSV[y, x]
        print('H:', values[0], '\tS:', values[1], '\tV:', values[2])


if __name__ == '__main__':
    # Grabs camera feed
    cv2.namedWindow('hsv_extractor')
    cv2.setMouseCallback('hsv_extractor', on_mouse, 0)

    while True:
        # Grab frame
        (grabbed, frame) = CAMERA.read()

        if not grabbed:
            break

        # Resize and convert to HSV
        frame_resized = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
        frame_hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

        IMG_DEBUG = frame_resized.copy()
        IMG_DEBUG_HSV = frame_hsv.copy()

        # Construct mask
        # Erode and dilate to remove inpurities
        mask = cv2.inRange(frame_hsv, RED_THRES_LOW, RED_THRES_HIGH)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours
        _, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        center = None

        # Only proceed if at least one contour was found
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), r) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if r > 5:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame_resized, (int(x), int(y)), int(r),
                           (0, 255, 255), 2)
                cv2.circle(frame_resized, center, 5, (0, 0, 255), -1)

        # Debug
        print(get_largest_blob_x_y(RED_THRES_LOW, RED_THRES_HIGH))
        cv2.imshow('Original', frame_resized)
        cv2.imshow('HSV', frame_hsv)
        cv2.imshow('Mask', mask)

        cv2.imshow('hsv_extractor', IMG_DEBUG)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

    CAMERA.release()
    cv2.destroyAllWindows()
