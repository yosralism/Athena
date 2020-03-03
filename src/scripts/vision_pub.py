#!/usr/bin/env python

import rospy
from athena.msg import Object
import imutils
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import cv2
import datetime
import time


def detection():
	pub = rospy.Publisher('cv_target', Object, queue_size=10)
	rospy.init_node('vision_pub', anonymous=True)
	rate = rospy.Rate(10)
        msg = Object()
	ap = argparse.ArgumentParser()
	ap.add_argument("-p", "--picamera", type=int, default=-1,
			help="whether or not the Raspberry Pi camera should be used")
	
	args = ap.parse_args(rospy.myargv()[1:])

	vs = VideoStream(usePiCamera=False ).start()
	time.sleep(2.0)
	
	while not rospy.is_shutdown():
		
		frame = vs.read()
		frame = imutils.resize(frame, width=400)
		#ret, frame = cap.read()
#		status = "No Targets"
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (11, 11), 0)
		thresh = cv2.threshold(blurred, 130, 255, cv2.THRESH_BINARY)[1]

	        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
			    cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		for c in cnts:
			# approximate the contour
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.01 * peri, True)
		 
			# ensure that the approximated contour is "roughly" rectangular
			if len(approx) >= 4 and len(approx) <= 6:
				# compute the bounding box of the approximated contour and
				# use the bounding box to compute the aspect ratio
				(x, y, w, h) = cv2.boundingRect(approx)
				aspectRatio = w / float(h)
		 
				# compute the solidity of the original contour
				area = cv2.contourArea(c)
				hullArea = cv2.contourArea(cv2.convexHull(c))
				solidity = area / float(hullArea)
		 
				# compute whether or not the width and height, solidity, and
				# aspect ratio of the contour falls within appropriate bounds
				keepDims = w > 25 and h > 25
				keepSolidity = solidity > 0.9
				keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2
		 
				# ensure that the contour passes all our tests
				if keepDims and keepSolidity and keepAspectRatio:
					# draw an outline around the target and update the status
					# text
					cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
#					status = "Target(s) Acquired"
	 
					# compute the center of the contour region and draw the
					# crosshairs
					M = cv2.moments(approx)
					(cX, cY) = (int(M["m10"] // M["m00"]), int(M["m01"] // M["m00"]))
					msg.x_obj = cX
	                		msg.y_obj = cY
					#rospy.ROS_INFO(cX, cY)	
#                                else:
#					cX = 0
#		                        cY = 0
#                                        msg.x_obj = cX
#                                        msg.y_obj = cY
        	
		#cv2.imshow("Frame", thresh)
		pub.publish(msg)
        	rate.sleep()
		
if __name__ == '__main__':
	try:
		detection()
	except rospy.ROSInterruptException:
		pass

