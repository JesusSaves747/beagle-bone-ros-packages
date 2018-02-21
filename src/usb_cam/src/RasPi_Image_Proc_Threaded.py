from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import time
import cv2
import numpy as np
from Perception_utils import custom_hough


num_frames = 100
display = True
minVal = 100
maxVal = 200

houghVoteThresh = 120
distThresh = 30  # pixels
segLengthThresh = 20 # pixels

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from `picamera` module...")

vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()

# loop over some frames...this time using the threaded stream
while fps._numFrames < num_frames:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	#frame = imutils.resize(frame, width=400)

	# # check to see if the frame should be displayed to our screen
	# if display:
	# 	cv2.imshow("Frame", frame)
	# 	key = cv2.waitKey(1) & 0xFF

    # Convert image to grayscale:
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Run Canny edge detector on the image:
    img_canny = cv2.Canny(img_gray, minVal, maxVal)

    # Run the custom Hough Transform:
    custom_hough(img_canny ,  houghVoteThresh, distThresh, segLengthThresh)


    print ( " ### --------------------------------####  ")

	# update the FPS counter
	fps.update()

fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
