#!/usr/bin/env python
from flask import Flask, render_template, Response
import cv2
from balloon_finder import BalloonFinder
app = Flask(__name__)

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen():
    """Video streaming generator function."""

    bf = BalloonFinder()
    while True:
        ###############################################
        # General usage example:

        # find full list of selected balloons.
        # and an image with them drawn on.
        im, balloon_list = bf.find_balloons()
        cv2.drawContours(im, balloon_list, -1, (255,0,0), 8)
        for b in balloon_list:
            # find the vector to that balloon
            tvec = bf.find_vector(b)

            if bf.is_definitely_balloon(b):
                (x,y), r = cv2.minEnclosingCircle(b)
                center = (int(x), int(y))
                rad = int(r)
                cv2.circle(im, center, rad,(0,255,0),2)

        bb = bf.pick_best_balloon(balloon_list)
        if bb != None:
            (x,y), r = cv2.minEnclosingCircle(bb)
            center = (int(x), int(y))
            rad = int(r)
            cv2.circle(im, center, rad,(0,0,255),8)
        #cv2.imshow('ball', im)
        #print "====Vector==================="
        #print np.array([tvec[0]*2.54, tvec[1]*2.54, tvec[2]*2.54])
        #print "============================="
        ###################################################


        #cv2.imshow('balloons', im)
        #for b in bloons:
        #    tvec = bf.find_vector(b)
        #    #tvec = bf.find_waypoint(gps_cord,b)
        #    print tvec
        #print "balloons: ", len(bloons)
       # cv2.imshow('canny ablloons', cann_im)
        
       # cv2.imshow('canny', cann)

        k = cv2.waitKey(1) & 0xFF
        if k ==27:
            break
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
