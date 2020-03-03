from imutils.video import FileVideoStream
from imutils.video import FPS
import numpy as np
import datetime
import argparse
import imutils
import time
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", required=True, help="path to input video file")
args = vars(ap.parse_args())


# start the file video stream thread and allow the buffer to
# start to fill
print("[INFO] starting video file thread...")
fvs = FileVideoStream(args["video"]).start()
time.sleep(1.0)

# start the FPS timer
fps = FPS().start()



while fvs.more():
	
	# grab the current frame and initialize the status text
	frame = fvs.read()
	frame = imutils.resize(frame, width=400)
	status = "No Targets"
 	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (13, 13), 0)
	thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
	
 
	# find contours in the edge map
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
				status = "Target(s) Acquired"
 
				# compute the center of the contour region and draw the
				# crosshairs
				M = cv2.moments(approx)
				(cX, cY) = (int(M["m10"] // M["m00"]), int(M["m01"] // M["m00"]))
				(startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
				(startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
				cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
				cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
				print(cX, cY)
			#else:
			#	cX = 0
			#	cY = 0
		
	#print(cX, cY)
	#cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
	#(0, 0, 255), 2)
 
	# show the frame and record if a key is pressed
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	fps.update()

 
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
fvs.stop()



