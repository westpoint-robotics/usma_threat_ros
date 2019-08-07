#!/usr/bin/env python

# Import required ros libraries code.
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf

# import python libraries
import numpy as np
import cv2
import os,sys
import yaml

class image_recorder():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# Ros params
		self.bridge = CvBridge()
		self.record_frame_rate = rospy.get_param("~frame_rate",1)
		self.save_frame_rate = rospy.Rate(self.record_frame_rate) # rospy.Rate(30) # 30hz

		self.image_topic = rospy.get_param("~image_topic","/pgr_1/image_raw")
		self.image_sub = rospy.Subscriber(self.image_topic,Image,self.record_ros_image)
		self.image_seq = 0
		print("Recording topic: {} ").format(self.image_topic)

		self.camera_namespace = rospy.get_param("~camera_namespace","pgr_1")
		self.folder_path = rospy.get_param("~folder_path","/")

		print("Saving images to: {}").format(self.folder_path)

	def record_ros_image(self,msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		image_filename = ("{}/{}_{:06d}.png").format(self.folder_path, self.camera_namespace, self.image_seq)
		print("saving image: {}").format(image_filename)
		try: 
			check = cv2.imwrite(image_filename, cv_image)
		except cv2.error as e:
			print(e)

		self.image_seq += 1
		self.save_frame_rate.sleep() # regulate the frame rate of the saving process
		


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('image_recorder')

	try:
		image_rec = image_recorder()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		rate.sleep()

