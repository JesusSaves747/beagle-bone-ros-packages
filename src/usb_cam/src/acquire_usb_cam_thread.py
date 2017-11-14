#!/usr/bin/env python
## Flowchart:
# 1. Create a class to measure FPS:
# 2. Create a class to start a threaded VideoCapture object: Use the Thread module from python:
import datetime
from threading import Thread
import cv2

# Import cv_bridge stuff:
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# license removed for brevity
import rospy
from std_msgs.msg import String
import time



# # 1.  Class to measure FRames per second as a metric for speed of video capture:
# class FPS:

#     # Define the constructor:
#     def __init__(self):
# 		# store the start time, end time, and total number of frames
# 		# that were examined between the start and end intervals
# 		self._start = None
# 		self._end = None
# 		self._numFrames = 0


#     # Define the methods:
#     def start(self):
# 		# start the timer
# 		self._start = datetime.datetime.now()
# 		return self

# 	def stop(self):
# 		# stop the timer
# 		self._end = datetime.datetime.now()

# 	def update(self):
# 		# increment the total number of frames examined during the
# 		# start and end intervals
# 		self._numFrames += 1

# 	def elapsed(self):
# 		# return the total number of seconds between the start and
# 		# end interval
# 		return (self._end - self._start).total_seconds()

# 	def fps(self):
# 		# compute the (approximate) frames per second
# 		return self._numFrames / self.elapsed()




# 2.
class WebcamVideoStream:

     # Constructor:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
		(self.grabbed, self.frame) = self.stream.read()

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

        def start(self):
            # start the thread to read frames from the video stream
            Thread(target=self.update, args=()).start()
            return self

        def update(self):
            # keep looping infinitely until the thread is stopped
            while True:
                # if the thread indicator variable is set, stop the thread
                if self.stopped:
                    return

                # otherwise, read the next frame from the stream
                (self.grabbed, self.frame) = self.stream.read()

        def read(self):
            # return the frame most recently read
            return self.frame

        def stop(self):
            # indicate that the thread should be stopped
            self.stopped = True



def img_publisher():

    global bridge


    # created a *threaded* video stream, allow the camera sensor to warmup,
    # and start the FPS counter
    #print("[INFO] sampling THREADED frames from webcam...")
    vs = WebcamVideoStream(src=1).start()

        # Create a publisher to the topic:
    rospy.init_node('USB_cam_image_thread', anonymous=True)
    pub_cam = rospy.Publisher('/ANRA/usb_cam/image_raw',Image , queue_size=10)

    print ("Image Acquisition Node Initialized")

    rate = rospy.Rate(60) # 10hz




    # loop over some frames...this time using the threaded stream
    while not rospy.is_shutdown():

        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 400 pixels
        frame = vs.read()


        try:
            pubStart = time.time()
            pub_cam.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            pubEnd = time.time()

            #print("Publishing took:", pubEnd - pubStart)

        except CvBridgeError as e:
            print(e)

        rate.sleep()


## Main function here:
if __name__ == '__main__':

    global bridge

    bridge = CvBridge()


    try:
        img_publisher()

    except rospy.ROSInterruptException:
        pass
