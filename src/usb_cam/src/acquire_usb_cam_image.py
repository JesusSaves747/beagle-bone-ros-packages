#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import roslib
import time
roslib.load_manifest('klt_tracker')

# Import cv_bridge stuff:
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# license removed for brevity
import rospy
from std_msgs.msg import String

def img_publisher():

    global bridge

    # Create a publisher to the topic:
    pub_cam = rospy.Publisher('/ANRA/usb_cam/image_raw',Image , queue_size=10 )

    # Open up a video capture object:
    camera = cv2.VideoCapture(1)   # I think 0 is the Laptop webcam and 1 is the USB cam




    rospy.init_node('USB_cam_image_pub', anonymous=True)
    print ("Image Acquisition Node Initialized")
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        # grab the current frame
        readStart = time.time()
        (grabbed, frame) = camera.read()
        readEnd = time.time()

        print(" Frame Read took: ", readEnd -readStart)


        # If the frame is grabbed then convert it to ROS Image Msg and publish:
        if grabbed:

            # resize the frame to 800 by 800:
            # frame = imutils.resize(frame, width=800, height =800)

            try:
                pubStart = time.time()
                pub_cam.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
                pubEnd = time.time()

                print("Publishing too:", pubEnd - pubStart)

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
