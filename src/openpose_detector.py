#!/usr/bin/env python

# cd ~/CMU/openpose/ && ./build/examples/openpose/openpose.bin --video examples/media/video.avi


# Import required Python code.
import math
# import sys
import sys;
import rospy
import cv2
import numpy as np
from usma_threat_ros.msg import ImageArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import tf

# import yaml
sys.path.append('/usr/local/python')
from openpose import pyopenpose as op

import tensorflow as tf
from feedforward_model import *

class openpose_detector():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# image array config
		self.bridge = CvBridge()
		self.threat_boxes_topic = rospy.get_param("~threat_image_topic","/threats/potential_images")
		self.threat_boxes_sub = rospy.Subscriber(self.threat_boxes_topic,ImageArray,self.threat_boxes)
		self.msg_seq = 0
		self.msg_time = 0
		self.skeletons = []

		# openpose config
		self.params = dict()
		self.params["model_folder"] = rospy.get_param("~model_folder_path","/cfg/models/")
		self.opWrapper = op.WrapperPython()
		self.opWrapper.configure(self.params)
		self.opWrapper.start()
		self.datum = op.Datum()

		# publish skeleton topics
		self.op_image_pub_topic = rospy.get_param("~skeleton_image_topic", "/threats/skeleton_images")
		self.op_image_pub = rospy.Publisher(self.op_image_pub_topic, Image, queue_size=10)
		self.op_image_seq = 0

		# threat model params
		self.threat_model_name = rospy.get_param("~threat_model_name", "default_model")
		self.threat_model_path = rospy.get_param("~threat_model_path")
		self.threat_model_meta = self.threat_model_path + self.threat_model_name + ".meta"
		# text params for writing threat labels
		fontFace = cv2.FONT_HERSHEY_DUPLEX
		text_color = (0,0,255)
		fontScale = 1
		text_thickness = 1
		self.textSize = cv2.getTextSize(classification, fontFace, fontScale, text_thickness);
		# print("openpose_detector::threat model = {}").format(self.threat_model_name)
		# print("openpose_detector::threat model path = {}").format(self.threat_model_path)
		# print("openpose_detector::threat model meta = {}").format(self.threat_model_meta)

	def threat_boxes(self,msg):
		if (self.msg_time != msg.header.stamp.to_sec()):
			self.msg_time = msg.header.stamp.to_sec()
			self.msg_seq += 1

			# Now loop over bboxes to see if any guns and people overlap
			i = 1
			for message_image in msg.images:
				try:
					image = self.bridge.imgmsg_to_cv2(message_image, "bgr8")
				except CvBridgeError as e:
					print(e)
					continue

				self.datum.cvInputData = image
				self.opWrapper.emplaceAndPop([self.datum])
				self.skeleton_image = self.datum.cvOutputData
				self.process_skeletons(self.datum.poseKeypoints, self.skeleton_image)
				self.publish_skeleton_image(self.skeleton_image)
				i+=1

				#do something with skeleton lists
				

	def process_skeletons(self, skeleton_list):
		x = np.empty((1,9,2))
		rele_dexes = [1,2,3,4,5,6,7,9,12]		
		right_elbow = 3
		right_wrist = 4

		if skeleton_list.size > 1:
			for joints in skeleton_list:
				if joints[right_elbow].all():
					joints[:,0:2] -= joints[right_elbow,0:2] # set right elbow as origin
					if joints[right_wrist].all():
						forearm_len = np.sqrt(joints[right_wrist][0]**2+joints[right_wrist][1]**2) # calculate pixel length of forearm
						joints[:,0:2] /= forearm_len # scale all joints by forearm length
						skele_x = joints[rele_dexes,0:2]
						# reshape for input to FFNN
						x = skele_x.reshape([1,skele_x.shape[0]*skele_x.shape[1]])
						classification = self.prediction(x)
						# print("!! classification: {}").format(classification)

						# text_width = textSize[0][0]
						# text_height = textSize[0][1]
						# text_baseline = textSize[1]
						# black_box_lower_left = (skele_image.shape[1]/2.0,skele_image.shape[0]/2.0)
						# black_box_upper_right = (black_box_lower_left[0]+text_width,black_box_lower_left[1]-text_height)
						# cv2.rectangle(self.skeleton_image,black_box_lower_left,black_box_upper_right,(0,0,0),-1)


	def prediction(self, skele):
		labels = ['high','med','low']
		session = tf.Session()
		with session as sess:
			# new_saver = tf.train.import_meta_graph('../threat_model/{}.meta'.format(model_name))
			new_saver = tf.train.import_meta_graph(self.threat_model_meta)
			# new_saver.restore(sess, '../threat_model/{}'.format(model_name))
			new_saver.restore(sess, self.threat_model_path + self.threat_model_name)
			graph = tf.get_default_graph()
			input_ph = graph.get_tensor_by_name("input_ph:0")
			pred = graph.get_tensor_by_name("pred:0")

			predictions = sess.run([pred],feed_dict={input_ph: skele})
			return labels[predictions[0][0]]

	def publish_skeleton_image(self, image_to_publish):
		try:
			image_msg = self.bridge.cv2_to_imgmsg(image_to_publish, "bgr8")
			image_msg.header.seq = self.op_image_seq
			image_msg.header.frame_id = "skeletons"
			self.op_image_pub.publish(image_msg)
			self.op_image_seq += 1
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('openpose_detector')

	try:
		skeleton = openpose_detector()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		rate.sleep()


	# imagename_i = "detected skeleton {}".format(i)
	# cv2.namedWindow(imagename_i, cv2.WINDOW_NORMAL)
	# cv2.imshow(imagename_i, self.datum.cvOutputData)
	# cv2.resizeWindow(imagename_i, image.shape[0]*2,image.shape[0]*2)
	# cv2.waitKey(25)