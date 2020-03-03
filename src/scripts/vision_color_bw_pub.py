#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time

from imutils import resize, grab_contours
from imutils.video import VideoStream
from athena.msg import Object

def color_detection():
    """
    Detect color with given threshold in HSV-space
    then take the largest blob
    """
    # init
    rospy.init_node('vision_pub', anonymous=True)
    pub = rospy.Publisher('cv_target', Object, queue_size=10)

    # get parameters
    threshold = rospy.get_param('~threshold', 130)

    rate = rospy.Rate(10)
    msg = Object()

    vs = VideoStream(usePiCamera=False).start()
    time.sleep(2.)

    while not rospy.is_shutdown():
        # preprocess
        frame = vs.read()
        frame = resize(frame, width=400)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.GaussianBlur(frame, (11, 11), 0)

        # color detection
        frame = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)[1]

        # find largest blob if exist
        # then compute the center
        cnts = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = grab_contours(cnts)

        if len(cnts) != 0:
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)

            try:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

            except ZeroDivisionError:
                continue

            msg.x_obj = cx
            msg.y_obj = cy

        # publish
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        color_detection()

    except rospy.ROSInterruptException:
        pass
