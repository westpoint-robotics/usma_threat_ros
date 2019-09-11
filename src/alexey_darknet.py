#!/usr/bin/env python

# Import required ros libraries code.
import rospy
from usma_threat_ros.msg import ImageArray, BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf

# import python libraries
import numpy as np
import cv2
import os,sys
import yaml

#setup and import darknet-yolo
sys.path.append('/usr/local/python') # path for CMUopenpose library
sys.path.append(os.environ['DARKNET_PATH']) 
sys.path.append(os.environ['DARKNET_PATH']+'/python')

print("sys.path.append({}) ").format(os.environ['DARKNET_PATH'])
print("sys.path.append({}) ").format(os.environ['DARKNET_PATH']+'/python')

import darknet as dn

class yolo_darknet():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# Darknet params
		self.darknet_cfg = rospy.get_param("~darknet_yolo/cfg")
		self.darknet_data = rospy.get_param("~darknet_yolo/data")
		self.darknet_weights = rospy.get_param("~darknet_yolo/weights")

		self.net = dn.load_net(self.darknet_cfg,self.darknet_weights,0)
		self.meta = dn.load_meta(self.darknet_data)

		print("    self.darknet_cfg:     {}").format(self.darknet_cfg)
		print("    self.darknet_data:    {}").format(self.darknet_data)
		print("    self.darknet_weights: {}").format(self.darknet_weights)

		# Ros params
		self.bridge = CvBridge()
		self.bb_image_pub_topic = rospy.get_param("~publishers/detection_image/topic")
		self.bb_image_pub = rospy.Publisher(self.bb_image_pub_topic, Image, queue_size=10)
		self.bb_image_seq = 0

		# params for adding text to bounding boxes
		self.fontFace = cv2.FONT_HERSHEY_DUPLEX
		self.fontScale = 0.5
		self.text_thickness = 1
		self.box_thickness = 2

		self.bounding_boxes_msg = BoundingBoxes()
		self.bounding_boxes_msg_seq = 0
		self.bounding_boxes_topic = rospy.get_param("~publishers/bounding_boxes/topic")
		self.bounding_boxes_pub = rospy.Publisher(self.bounding_boxes_topic, BoundingBoxes, queue_size=10)

		self.image_topic = rospy.get_param("~subscribers/image/topic")
		self.image_sub = rospy.Subscriber(self.image_topic,Image,self.detect_ros_image)

	def detect_ros_image(self,msg):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			self.image_rx = True
		except CvBridgeError as e:
			print(e)

		frame_resized = cv2.resize(self.cv_image,(dn.network_width(self.net),dn.network_height(self.net)),interpolation=cv2.INTER_LINEAR)
		darknet_image = dn.make_image(dn.network_width(self.net),dn.network_height(self.net),3)
		dn.copy_image_from_bytes(darknet_image,frame_resized.tobytes())
		self.detections = dn.detect_image(self.net, self.meta, darknet_image, thresh=0.25)

		height_scale = float(self.cv_image.shape[0]) / dn.network_height(self.net)
		width_scale = float(self.cv_image.shape[1]) / dn.network_height(self.net)

		self.bounding_boxes_msg = BoundingBoxes()
		box_i = 0
		for detection in self.detections:
			if (detection[0] == "pistol"):
				box_color = (255,0,0)
			elif (detection[0] == "person"):
				box_color = (0,0,255)
			else:
				box_color = (0,255,0)

			box = BoundingBox()
			box.Class = detection[0]
			box.probability = detection[1]
			bounds = detection[2]
			box.xmin = (bounds[0] - bounds[2]/2)
			box.ymin = (bounds[1] - bounds[3]/2)
			box.xmax = (bounds[0] + bounds[2]/2)
			box.ymax = (bounds[1] + bounds[3]/2)
			cv2.rectangle(frame_resized,(int(box.xmin),int(box.ymin)),(int(box.xmax),int(box.ymax)),box_color,2)
			frame_resized = self.add_label_to_box(detection[0], box, box_color, frame_resized)

			box.xmin *= width_scale
			box.ymin *= height_scale
			box.xmax *= width_scale
			box.ymax *= height_scale
			self.bounding_boxes_msg.bounding_box.append(box)
			

		self.publish_boxes_array()
		# this publishes the image with bounding boxes
		self.publish_image_with_boxes(cv2.resize(frame_resized,(self.cv_image.shape[1], self.cv_image.shape[0]),interpolation=cv2.INTER_LINEAR)	)

	def add_label_to_box(self, label, box, text_color, cv_image):
		textSize = cv2.getTextSize(label, self.fontFace, self.fontScale, self.text_thickness);
		text_width = textSize[0][0]
		text_height = textSize[0][1]
		text_baseline = textSize[1]

		black_box_lower_left = (int(box.xmin) + self.box_thickness,  int(box.ymin) + text_height + text_baseline + self.box_thickness)
		black_box_upper_right = (int(box.xmin) + self.box_thickness + text_width,  int(box.ymin)+self.box_thickness)
		cv2.rectangle(cv_image,black_box_lower_left,black_box_upper_right,(0,0,0),-1)

		text_lower_left = (int(box.xmin) + self.box_thickness,  int(box.ymin) + text_height+self.box_thickness)
		text_upper_right = (int(box.xmin) + self.box_thickness+text_width,  int(box.ymin) + self.box_thickness)
		cv2.putText(cv_image, label, text_lower_left, self.fontFace, self.fontScale, text_color, self.text_thickness,cv2.LINE_AA)
		return cv_image

	def publish_boxes_array(self):
		self.bounding_boxes_msg.header.stamp = rospy.Time.now()
		self.bounding_boxes_msg.header.seq = self.bounding_boxes_msg_seq
		self.bounding_boxes_msg.header.frame_id = "bounding boxes"

		self.bounding_boxes_pub.publish(self.bounding_boxes_msg)
		self.bounding_boxes_msg_seq += 1

	def publish_image_with_boxes(self, image_to_publish):
		try:
			image_msg = self.bridge.cv2_to_imgmsg(image_to_publish, "bgr8")
			image_msg.header.seq = self.bb_image_seq
			image_msg.header.frame_id = "bounding boxes"
			self.bb_image_pub.publish(image_msg)
			self.bb_image_seq += 1
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('yolo_darknet')

	try:
		yolo = yolo_darknet()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		rate.sleep()

