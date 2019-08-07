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
		
		self.image_topic = rospy.get_param("~image_topic","/pgr_1/image_raw")
		self.image_sub = rospy.Subscriber(self.image_topic,Image,self.record_ros_image)
		self.image_seq = 0
		print("Recording topic: {} ").format(self.image_topic)

		self.camera_namespace = rospy.get_param("~camera_namespace","pgr_1")
		self.folder_path = rospy.get_param("~folder_path","/")

		print("Saving {} images to: {}").format(self.camera_namespace, self.folder_path)

	def record_ros_image(self,msg):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			image_filename = ("saving image: {}/{}_{:06d}.png").format(self.folder_path, self.camera_namespace, self.image_seq)
			print("{}").format(image_filename)
			cv2.imwrite(image_filename,self.cv_image)
			# bool check = cv2.imwrite("./img.bmp", reflection);

			self.image_seq += 1
		except CvBridgeError as e:
			print(e)



if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('image_recorder')

	try:
		image_rec = image_recorder()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		rate.sleep()

