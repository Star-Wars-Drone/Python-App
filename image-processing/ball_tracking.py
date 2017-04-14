# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
#from collections import deque
#import numpy as np
#import argparse
#import imutils
#import cv2


# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-v", "--video",
#	help="path to the (optional) video file")
#ap.add_argument("-b", "--buffer", type=int, default=64,
#	help="max buffer size")
#args = vars(ap.parse_args())

# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points
#redLower = (0, 100, 100)
#redUpper = (50, 255, 255)
#pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
#if not args.get("video", False):
#	camera = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
#else:
#	camera = cv2.VideoCapture(args["video"])

# keep looping
#while True:
	# grab the current frame
#	(grabbed, frame) = camera.read()

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
#	if args.get("video") and not grabbed:
#		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
#	frame = imutils.resize(frame, width=600)
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
#	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
#	mask = cv2.inRange(hsv, redLower, redUpper)
#	mask = cv2.erode(mask, None, iterations=2)
#	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
#	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
#		cv2.CHAIN_APPROX_SIMPLE)[-2]
#	center = None

	# only proceed if at least one contour was found
#	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
#		c = max(cnts, key=cv2.contourArea)
#		((x, y), radius) = cv2.minEnclosingCircle(c)
#		M = cv2.moments(c)
#		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
#		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
#			cv2.circle(frame, (int(x), int(y)), int(radius),
#				(0, 255, 255), 2)
#			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# update the points queue
#	pts.appendleft(center)

	# loop over the set of tracked points
#	for i in xrange(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
#		if pts[i - 1] is None or pts[i] is None:
#			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
#		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
#		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
#	cv2.imshow("Frame", frame)
#	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
#	if key == ord("q"):
#		break

# cleanup the camera and close any open windows
#camera.release()
#cv2.destroyAllWindows()

import cv2
import numpy as np





# out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

# Basically, use these things to look for balloon looking things.
# http://docs.opencv.org/3.2.0/d1/d32/tutorial_py_contour_properties.html

# hsv red is [[[  0 255 255]]]



#low_red = np.array([300, 70, 70])
#upper_red = np.array([360, 255, 255])


class BalloonFinder(object):
  # pixels_to_angle_x - converts a number of pixels into an angle in radians 
	def pixels_to_angle_x(self, num_pixels):
		return num_pixels * math.radians(self.cam_hfov) / self.img_width
    
    # pixels_to_angle_y - converts a number of pixels into an angle in radians 
	def pixels_to_angle_y(self, num_pixels):
		return num_pixels * math.radians(self.cam_vfov) / self.img_height
	
	def get_distance_from_pixels(self,size_in_pixels, actual_size):
		if (size_in_pixels == 0):
			return 9999.9
    # convert num_pixels to angular size
		return actual_size / self.pixels_to_angle_x(size_in_pixels)

	def angle_to_pixels_x(self, angle):
		return int(angle * self.img_width / math.radians(self.cam_hfov))

	def project_position(self, origin, pitch, yaw, distance):
		cos_pitch = math.cos(pitch)
		dx = distance * math.cos(yaw) * cos_pitch
		dy = distance * math.sin(yaw) * cos_pitch
		dz = distance * math.sin(pitch)
		ret = PositionVector(origin.x + dx,origin.y + dy, origin.z + dz)
		return ret

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
	if(len(bloons)>0):
		contour=bloons[0]
		(x,y), (MA,ma), angle = cv2.fitEllipse(contour)
		ellipse = cv2.fitEllipse(contour)
		eps = 0.1*cv2.arcLength(contour,True)
		aprx = cv2.approxPolyDP(contour, eps, True)
		area = cv2.contourArea(aprx)
		radius = MA/2
		print "radius : ", radius
		distance=bf.get_distance_from_pixels(radius, 0.2)
		bf.project_position(0,0,0,distance)
		
	k = cv2.waitKey(5) & 0xFF
	if k ==27:
		break

