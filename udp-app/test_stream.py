#!/usr/bin/env python
from flask import Flask, render_template, Response
import cv2
from balloon_video import BalloonFinder
app = Flask(__name__)
#vc = cv2.VideoCapture(0)
bf = BalloonFinder()
im, balloon_list = bf.find_balloons()

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen():
    """Video streaming generator function."""
    while True:
        
        ret, im = bf.cam.read()
        real_im = im
        mask = bf.filter_and_mask(im)
        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        balloons = []
        for c in cnts:
            #peri = cv2.arcLength(c, True)
            #approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if bf.is_balloon(c):
                cv2.drawContours(im, [c], 0, (255,0,0), 7)
                balloons.append(c) 
            if bf.is_definitely_balloon(c):
                (x,y), r = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                rad = int(r)
                cv2.circle(im, center, rad,(0,255,0),2)

        cv2.imwrite('t.jpg', im)
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + open('t.jpg', 'rb').read() + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=False, threaded=True)
