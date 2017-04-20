import cv2
import numpy as np
import socket
import sys
import pickle
cap=cv2.VideoCapture(0)
clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('192.168.1.109',8089))
while True:
    ret,frame=cap.read()
    print sys.getsizeof(frame)
    print frame
    clientsocket.sendall(pickle.dumps(frame))
