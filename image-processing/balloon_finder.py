import cv2
import numpy as np





# out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

# Basically, use these things to look for balloon looking things.
# http://docs.opencv.org/3.2.0/d1/d32/tutorial_py_contour_properties.html

# hsv red is [[[  0 255 255]]]



#low_red = np.array([300, 70, 70])
#upper_red = np.array([360, 255, 255])


class BalloonFinder(object):

    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.im_cnt =0;
        self.vid_cnt = 0;
        self.low_red = np.array([0, 100, 100])
        self.upper_red = np.array([255, 255, 255])

    def save_image(self):
        ret, im = self.cam.read()
        filename = 'IM_CAP_' + str(self.im_cnt) + '.jpg'
        self.im_cnt = self.im_cnt + 1
        cv2.imwrite(filename, im)
        
    def save_video(self, length=3):
        """saves a 1 min video by default"""
        #thread.start()
        t0 = time.time()
        filename = 'VID_CAP_' + str(self.vid_cnt) + '.avi'

        # test frame
        ret, frame = self.cam.read()     
        height, width, ch = frame.shape  

        self.vid_cnt = self.vid_cnt+1
        fourcc = cv2.cv.CV_FOURCC(*'XVID')
        vid_writer = cv2.VideoWriter(filename,fourcc, 20, (width,height)) 
        while (time.time() - length) < t0:
            ret, frame = self.cam.read()
            vid_writer.write(frame)
        vid_writer.release()



    def is_solid(self, contour):
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


    def is_round(self, contour):
        """checks that contour is many edged"""

        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02*peri, True)

        # expect very large # of edges
        if len(approx) > 5:
            return True
        return False    

    def is_elliptical(self, contour):
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
            #cv2.line(im, (int(x), int(y)), (int(x),int(y)+int(MA/2)), (255,0,255))
            #cv2.line(im, (int(x), int(y)), (int(x)-int(ma/2),int(y)),(255,0,255))
        else:
            return False

        #string = "ell: {0:0.2f}, area: {0:0.2f},\n ratio: {0:0.2f}".format(ell_area, area, ratio)
        #string = "ratio: {0:0.2f}".format(ratio)
        #cv2.ellipse(im, ellipse, (0,0,255),2)
        #cv2.putText(im, string, (int(x),int(y)), font, 1, (0,255,0), 1)
        #cv2.putText(im,'OpenCV',(10,500), font, 4,(255,255,255),2)



        if 1.4 < ratio < 1.6:
            #cv2.putText(im, "Balloon", (int(x),int(y)), font, 1, (255,0,255), 9)
            return True
        else:
            return False


    def is_balloon(self, contour):
        
        sld = self.is_solid(contour)
        rnd = self.is_round(contour)
        ep = self.is_elliptical(contour)
        
        return (ep and sld and rnd)



    def filter_and_mask(self, frame):
        #blur = cv2.GaussianBlur(frame,(0,0),3)
        cv2.imshow('filtering', frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low_red, self.upper_red)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def find_balloons(self):
        """finds list of all balloon-like contours in image
            recommended to use an image that has masked out all non-red
        """

        ret, im = self.cam.read()

        mask = self.filter_and_mask(im)
        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        balloons = []
        
        #TODO(Ahmed): figure out how to make this useful.
        for c in cnts:
            #peri = cv2.arcLength(c, True)
            #approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if self.is_balloon(c):
                #cv2.drawContours(image, [c], 0, (255,0,0), 8)
                balloons.append(c) 
        return balloons



bf = BalloonFinder()
while True:
    bloons = bf.find_balloons()
    print "balloons: ", len(bloons)
    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break

