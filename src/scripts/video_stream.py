from threading import Threading
import cv2

class video_stream:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()
 
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

