import cv2
import numpy as np




cam = cv2.VideoCapture(0)
detector = cv2.SimpleBlobDetector()


# out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

# Basically, use these things to look for balloon looking things.
# http://docs.opencv.org/3.2.0/d1/d32/tutorial_py_contour_properties.html

# hsv red is [[[  0 255 255]]]
low_red = np.array([0, 100, 100])
upper_red = np.array([255, 255, 255])


#low_red = np.array([300, 70, 70])
#upper_red = np.array([360, 255, 255])


def hsv_range(frame):
    """tells lowest and highest h-s-v values of an obj"""

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, d = frame.shape

    hsv_lower = hsv[0,0]
    hsv_upper = [0, 0, 0]
 
    for i in range(0, height):
        for j in range(0, width):
            i=0        



def redout(frame):
    height, width, depth = frame.shape
    for i in range(0, height):
        for j in range(0, width):
            #check if mostly red
            # BGR, B=0, G=1, R=3
            if frame[i,j,0] > frame[i,j,2] and frame[i,j,1] > frame[i,j,2]:
                frame[i,j] = [0,0,0]

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
        return True
    return False    

font = cv2.FONT_HERSHEY_SIMPLEX

def is_elliptical(contour):
    """git an ellipse, check if that ellipse is still close"""

    if len(contour) < 5:
        return False
    (x,y), (MA,ma), angle = cv2.fitEllipse(contour)
    ellipse = cv2.fitEllipse(contour)

    ell_area = 3.14159264*(MA/2)*(ma/2)

    eps = 0.1*cv2.arcLength(contour,True)
    aprx = cv2.approxPolyDP(contour, eps, True)
    area = cv2.contourArea(aprx)

    if area > 300:  
        ratio = float(ell_area)/float(area)
        string = "area: " + str(area) + "ell_area: " + str(ell_area)
        #cv2.putText(im, string, (int(x),int(y)), font, 1, (0,255,0), 1)     
        cv2.line(im, (int(x), int(y)), (int(x),int(y)+int(MA/2)), (255,0,255))
        cv2.line(im, (int(x), int(y)), (int(x)-int(ma/2),int(y)),(255,0,255))
    else:
        return False

    #string = "ell: {0:0.2f}, area: {0:0.2f},\n ratio: {0:0.2f}".format(ell_area, area, ratio)
    #string = "ratio: {0:0.2f}".format(ratio)
    cv2.ellipse(im, ellipse, (0,0,255),2)
    #cv2.putText(im, string, (int(x),int(y)), font, 1, (0,255,0), 1)
    #cv2.putText(im,'OpenCV',(10,500), font, 4,(255,255,255),2)



    if 1.4 < ratio < 1.6:
        cv2.putText(im, "Balloon", (int(x),int(y)), font, 1, (255,0,255), 9)
        return True
    else:
        return False


def is_balloon(contour):
    
    sld = is_solid(contour)
    rnd = is_round(contour)
    ep = is_elliptical(contour)
    
    return (ep and sld and rnd)

def filter_and_mask(frame):
    frame = cv2.GaussianBlur(frame,(0,0),3)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #hsv = cv2.GaussianBlur(hsv,(0,0),3)
    mask = cv2.inRange(hsv, low_red, upper_red)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask

def find_balloons(image):
    """finds list of all balloon-like contours in image
        recommended to use an image that has masked out all non-red
    """
    cnts = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    balloons = []
    
    #TODO(Ahmed): figure out how to make this useful.
    for c in cnts:
        #peri = cv2.arcLength(c, True)
        #approx = cv2.approxPolyDP(c, 0.02*peri, True)
        if is_balloon(c):
            #cv2.drawContours(image, [c], 0, (255,0,0), 8)
            balloons.append(c) 
    return balloons

while True:
    ret, im = cam.read()

    #cv2.imshow('raw', im)
    #blob_points = detector.detect(im)
    # im_with_blobs = cv2.drawKeypoints(im, blob_points, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow('points', im_with_blobs)
    cv2.imshow('none', im)
    im = cv2.GaussianBlur(im,(0,0),3)

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #hsv = cv2.GaussianBlur(hsv,(0,0),3)
    mask = cv2.inRange(hsv, low_red, upper_red)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)


    #cv2.imshow('mask', mask)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]


    cv2.drawContours(im, cnts, -1, (0,255,0), 3)

    for c in cnts:
        #peri = cv2.arcLength(c, True)
        #approx = cv2.approxPolyDP(c, 0.02*peri, True)
        if is_balloon(c):
            cv2.drawContours(im, [c], 0, (255,0,0), 8)
    #frame = cv2.bitwise_and(frame,frame,mask=mask)
    #gray = cv2.cvtColor(frame, cv2.COLOR_HSV2GRAY)
   
    #ret, thresh = cv2.threshold(gray, 127, 255, 0)


    cv2.imshow('filtered', im)

    # TODO(Ahmed) Consider using contour approx before doing this to 'close' shapes
    # This may ruin everything actually.

    #for c in contours:
    #    if is_balloon(c):
    #        cv2.drawContours(im, c, -1, (255,0,0), 3)

    #cv2.imshow('cnts', im)

    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break

