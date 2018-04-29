import numpy as np
import cv2
cam=cv2.VideoCapture(1)


while True:
    _,im=cam.read()
    p=cv2.cvtColor(im, cv2.COLOR_BGR2LUV)
    p[:,:,0]=0
    #p=cv2.inRange(p,(0,150,0),(15,250,255))
    #p = cv2.erode(p, None, iterations=2)
    #p = cv2.dilate(p, None, iterations=2)
    
    #_, cnts, _ = cv2.findContours(p, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #if len(cnts) > 1:
    #    c = max(cnts, key=cv2.contourArea)
    #    ((x, y), r) = cv2.minEnclosingCircle(c)
    #    # only proceed if the radius meets a minimum size
    #    if r > 5:
    #        # draw the circle and centroid on the frame,
    #        # then update the list of tracked points
    #        cv2.circle(im, (int(x), int(y)), int(r),(0, 255, 255), 2)
    cv2.imshow("im",im)
    cv2.imshow("p",(p*250))
    
    cv2.waitKey(1)
