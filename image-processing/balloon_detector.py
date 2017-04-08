import cv2
import numpy as np


cam = cv2.VideoCapture(0)
detector = cv2.SimpleBlobDetector()


def filters(frame):
    # gray and blur
    frame = cv2.GaussianBlur(frame,(0,0),3)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 2)
    # ret, frame = cv2.threshold(frame, 127, 255, 0)


    return frame


def is_solid(contour):
    """ Checks to see if the contour is "solid".
    hard to describe in text but google convexHull"""

    area = cv2.contourArea(contour)
    hullArea = cv2.contourArea(cv2.convexHull(contour))
    if hullArea > 0:
        solidity = area / float(hullArea)
    else:
        return False

    solid = solidity  > 0.9
    return solid

def is_round(contour):
    """checks that contour is many edged"""

    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.02*peri, True)

    # expect very large # of edges
    if len(approx) > 5:
        print "found round"
        return True
    return False    

def is_elliptical(contour):
    """git an ellipse, check if that ellipse is still close"""

    ellipse = cv2.fitEllipse(contour)
    eps = 0.1*cv2.arcLength(contour,True)

    aprx = cv2.approxPolyDP(contour, eps, True)

    ratio = cv2.contourArea(ellipse)/cv2.contourArea(aprx)  
    return 0.8 < ratio < 1.2


def is_balloon(contour):
    
    sld = is_solid(contour)
    rnd = is_round(contour)
    ep = is_elliptical(contour)

    return sld and rnd and ep




while True:
    ret, im = cam.read()

    #cv2.imshow('raw', im)
    #blob_points = detector.detect(im)
    # im_with_blobs = cv2.drawKeypoints(im, blob_points, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow('points', im_with_blobs)

    frame = filters(im)
    cv2.imshow('filtered', frame)
    
    contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(im, contours, -1, (0,255,0), 3)

    # TODO(AHmed) Consider using contour approx before doing this to 'close' shapes

    for c in contours:
        if is_balloon(c):
            cv2.drawContours(im, c, -1, (255,0,0), 3)

    cv2.imshow('cnts', im)

    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break

