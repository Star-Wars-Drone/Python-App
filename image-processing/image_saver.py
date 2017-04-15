import threading, time

import numpy as np
import cv2




class ImageSaver(object):

    
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.im_cnt =0;
        self.vid_cnt = 0;

        #self.thread = threading.Thread(target=self.save_video, args(self, time)

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
           

ims = ImageSaver()

ims.save_image()

ims.save_video()
