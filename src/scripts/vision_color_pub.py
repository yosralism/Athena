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
    lh = rospy.get_param('~lowerH', 0)
    ls = rospy.get_param('~lowerS', 0)
    lv = rospy.get_param('~lowerV', 0)

    uh = rospy.get_param('~upperH', 179)
    us = rospy.get_param('~upperS', 255)
    uv = rospy.get_param('~upperV', 255)

    assert (lh < uh) and (ls < us) and (lv < uv),\
        "lower-threshold should be lower than upper-threshold"
    assert (0 <= lh <= 179) and (0 <= uh <= 179),\
        "Hue value should be in range 0-179"
    assert (0 <= ls <= 255) and (0 <= us <= 255),\
        "Hue value should be in range 0-179"
    assert (0 <= lv <= 255) and (0 <= uv <= 255),\
        "Hue value should be in range 0-179"

    lower = np.array([lh, ls, lv], dtype='uint8')
    upper = np.array([uh, us, uv], dtype='uint8')

    rate = rospy.Rate(10)
    msg = Object()

    vs = VideoStream(usePiCamera=False).start()
    time.sleep(2.)

    while not rospy.is_shutdown():
        # preprocess
        frame = vs.read()
        frame = resize(frame, width=400)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame = cv2.GaussianBlur(frame, (11, 11), 0)

        # color detection
        frame = cv2.inRange(frame, lower, upper)

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
