#!/usr/bin/env python

import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(data):
	bridge = CvBridge()

	current_img  = bridge.imgmsg_to_cv2(data,desired_encoding='bgra8')

	resized = cv2.resize(current_img, (0, 0), fx = 0.6, fy = 0.6)

	cv2.imshow("video", resized)
	cv2.waitKey(1)

def video_subscriber():
	rospy.init_node("video_subscriber_py", anonymous=True)
	rospy.loginfo("Starting Video Frames")

	rospy.Subscriber("/camera/image_raw", Image, callback)

	rospy.spin()

	cv2.destroyAllWindows()

if __name__ ==  "__main__":
	video_subscriber()