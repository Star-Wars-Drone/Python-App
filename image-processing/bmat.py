import cv2
import numpy as np


cam = cv2.VideoCapture(0)
detector = cv2.SimpleBlobDetector()





im = cv2.imread('ahmed.jpg')

x,y = (im > 9000).nonzero()
vals = im[x,y]


