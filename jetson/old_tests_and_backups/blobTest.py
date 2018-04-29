import numpy as np
import cv2
cam=cv2.VideoCapture(1)


ratio=3/3
while True:
    _,im=cam.read()
    p=im.astype("int16")
    p=p[:,:,2]-p[:,:,0]*ratio-p[:,:,1]*ratio
    p=p*5
    p=np.maximum(np.minimum(p,255),0)
    p=p.astype("uint8")

    p = cv2.dilate(p, None, iterations=2)
    p = cv2.erode(p, None, iterations=2)
    
    _, cnts, _ = cv2.findContours(p.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts) > 1:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), r) = cv2.minEnclosingCircle(c)
        # only proceed if the radius meets a minimum size
        if r > 5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(im, (int(x), int(y)), int(r),(0, 255, 255), 2)
    cv2.imshow("p",p)
    cv2.imshow("im",im)
    cv2.waitKey(1)
