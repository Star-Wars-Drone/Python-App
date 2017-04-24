import cv2
import numpy as np
import math




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
        
        # thresh from drone cam
        #self.low_red = np.array([124,106,163])
        #self.upper_red = np.array([194,255,255])


        #TODO(Ahmed): Replace with actual valuesi.
        self.balloon_mat = np.float32([[4,0,0],
                                       [-4,0,0],
                                       [0,5,0],
                                       [0,-5,0]])

        self.cam_matrix = np.zeros((3,3), np.float32)

        self.cam_matrix[0,0] = 7.6292554546337738e+02
        self.cam_matrix[0,1] = 0.0
        self.cam_matrix[0,2] = 3.1950000000000000e+02
        self.cam_matrix[1,0] = 0. 
        self.cam_matrix[1,1] = 7.6292554546337738e+02
        self.cam_matrix[1,2] = 2.3950000000000000e+02
        self.cam_matrix[2,0] = 0.
        self.cam_matrix[2,1] = 0.
        self.cam_matrix[2,2] = 1.

        self.distcoeffs = np.zeros((1,5), np.float32)
        self.distcoeffs[0,0] = -4.5614114539630291e-01
        self.distcoeffs[0,1] = 8.8158732627801784e-01
        self.distcoeffs[0,2] = 0.
        self.distcoeffs[0,3] = 0.
        self.distcoeffs[0,4] = -2.3488206318160914e+00

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
        if len(approx) > 15:
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
        
        return (ep and rnd)


    def filter_and_mask(self, frame):
        #blur = cv2.GaussianBlur(frame,(0,0),3)
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
        
        cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        balloons = []
        

 

        #TODO(Ahmed): figure out how to make this useful.
        for c in cnts:

            #peri = cv2.arcLength(c, True)
            #approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if self.is_balloon(c):
                #cv2.drawContours(image, [c], 0, (255,0,0), 8)
                balloons.append(c) 
        return im, balloons

    def get_lower_half(self, cnt):
        """ removes the upper half of a contour.
            Midpoint calculated based on bounding ellipse

            Based on this:
            http://stackoverflow.com/questions/27208532/is-there-a-way-to-remove-points-from-a-contour-outside-a-given-circle-in-python
        """

        (rx,ry), (MA,ma), angle = cv2.fitEllipse(cnt)

        # This is how you extract points from contour
        #x = p[0][0]
        #y = p[0][1]
        #recall that 0,0 is top left.
        raw_cnt = np.squeeze(cnt)
        mask = raw_cnt[:,1] < ry

        low_half = raw_cnt[mask,:]
        return low_half

    def extreme_points(self, cnt):
        left = tuple(cnt[cnt[:,:,0].argmin()][0])
        right = tuple(cnt[cnt[:,:,0].argmax()][0])
        top = tuple(cnt[cnt[:,:,1].argmin()][0])
        bottom = tuple(cnt[cnt[:,:,1].argmax()][0])
        return np.float32([left, right, top, bottom])

    def find_vector(self, bloon_cnt):
        outline = self.extreme_points(bloon_cnt)
        
        ret, rvec, tvec = cv2.solvePnP(self.balloon_mat, outline, self.cam_matrix, self.distcoeffs)
 
        return tvec

    def pick_best_balloon(self, balloon_list):
        """recieves list of contours it suspects to be balloons and determines which one is
        most likely to be the true balloon."""

        b = max(balloon_list, key=cv2contourArea)
        
        return b
        
    
    def find_waypoint(self,current_gps_location,bloon_cnt):
        tvec = self.find_vector(bloon_cnt)
        #spherical model of the earth
        #lat, long in radians and altitude in meters
        current_lat = current_gps_location[0]
        current_lon = current_gps_location[1]
        current_alt = current_gps_location[2]
        r = 6371000 + current_alt
        current_x = r*math.cos(current_lat)*math.cos(current_lon)
        current_y = r*math.cos(current_lat)*math.sin(current_lon)
        current_z = r*math.sin(current_lat)
        new_x = current_x + tvec[0]
        new_y = current_y + tvec[1]
        new_z = current_z + tvec[2]
        new_lon = math.atan2(new_y,new_x)
        lat_cal_denom = math.sqrt((new_x*new_x) + (new_y*new_y))
        new_lat = math.atan2(new_z,lat_cal_denom)
        new_r = math.sqrt((new_x*new_x)+(new_y*new_y)+(new_z*new_z))
        new_altitude = new_r-6371000
        waypoint=[new_lat,new_lon,new_altitude]
        return waypoint



def main():
    bf = BalloonFinder()
    while True:
        ###############################################
        # General usage example:

        # Dummy value for laptp testing.
        gps_cord=[0.624371939852,-1.37329479882,30]   
        # find full list of selected balloons.
        # and an image with them drawn on.
        im, balloon_list = bf.find_balloons()
        cv2.drawContours(im, balloon_list, -1, (255,0,0), 8)


        cv2.imshow('cnts', im)
        for b in balloon_list:
            # find the vector to that balloon
            tvec = bf.find_vector(b)
            low_h = bf.get_lower_half(b)
            cv2.drawContours(im, [low_h], -1, (0,0,255),8)
            # calculate waypoint to balloon
                
            print "====Vector==================="
            print np.array([tvec[0]*2.54, tvec[1]*2.54, tvec[2]*2.54])
            print "============================="
            ###################################################


        #for b in bloons:
        #    tvec = bf.find_vector(b)
        #    #tvec = bf.find_waypoint(gps_cord,b)
        #    print tvec
        #print "balloons: ", len(bloons)
       # cv2.imshow('canny ablloons', cann_im)
        
       # cv2.imshow('canny', cann)

        k = cv2.waitKey(5) & 0xFF
        if k ==27:
            break


if __name__ == '__main__':
    main()
