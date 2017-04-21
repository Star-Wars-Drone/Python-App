
import numpy as np
import cv2
import zmqimage
import time
print "Connecting to zmqShowImage Server ... "
zmq = zmqimage.zmqConnect()
#image = np.zeros((500, 500), dtype="uint8")
cap = cv2.VideoCapture(0)
while(cap.isOpened()):
    ret, frame = cap.read()
    print "reading frame"
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    zmq.imshow("Zero Image 500 x 500", res)
    time.sleep(.033)
# build a rectangular mask & display it
mask = np.zeros(image.shape[:2], dtype="uint8")
cv2.rectangle(mask, (0, 90), (300, 450), 255, -1)
zmq.imshow("Rectangular Mask", mask)
