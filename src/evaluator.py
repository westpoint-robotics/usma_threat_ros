#!/usr/bin/env python

# Import required Python code.
import math
import sys
import rospy
import cv2
from usma_threat_ros.msg import ImageArray, BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import tf

class box_evaluator():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# ros config
		self.yolo_boxes_topic = rospy.get_param("~yolo_boxes_topic","/darknet_ros/bounding_boxes")
		self.yolo_boxes_sub = rospy.Subscriber(self.yolo_boxes_topic,BoundingBoxes,self.yolo_boxes)
		self.yolo_boxes_list = []
		self.msg_seq = 0
		self.msg_time = 0
		self.human_type = "person"
		self.gun_type = "pistol"

		self.gun_boxes = []
		self.human_boxes = []
		self.pose_imgs = []
		self.potential_threats = []
		self.potential_threats_imgs = ImageArray()

		self.bridge = CvBridge()
		self.srcimg_topic = rospy.get_param("~srcimg_topic","/ardrone/front/image_rect_color")
		self.srcimg_sub = rospy.Subscriber(self.srcimg_topic,Image,self.updateimage)
		self.image_rx = False

		self.threat_img_topic = rospy.get_param("~threat_image_topic","/threats/potential_images")
		self.threat_img_pub = rospy.Publisher(self.threat_img_topic, ImageArray, queue_size=10)

	def updateimage(self,msg):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			self.image_rx = True
		except CvBridgeError as e:
			print(e)

	def yolo_boxes(self,msg):
		if (self.msg_time != msg.header.stamp.to_sec()):
			self.msg_time = msg.header.stamp.to_sec()
			self.msg_seq += 1
			self.yolo_box_data = msg.bounding_box

			# self.bboxes = {self.human_type:[], self.gun_type:[]}
			self.gun_boxes = []
			self.human_boxes = []
			self.potential_threats = []

			for box in self.yolo_box_data:
				if box.Class == self.human_type:
					self.human_boxes.append({'xmin':box.xmin,'ymin':box.ymin,'xmax':box.xmax,'ymax':box.ymax})

				if box.Class == self.gun_type:
					self.gun_boxes.append({'xmin':box.xmin,'ymin':box.ymin,'xmax':box.xmax,'ymax':box.ymax})

			# Now loop over bboxes to see if any guns and people overlap
			for person_box in self.human_boxes:
				for gun_box in self.gun_boxes:
					if self.box_overlap(person_box, gun_box):
						self.potential_threats.append(person_box)
			
			if self.image_rx:
				self.potential_threats_imgs = ImageArray()
				self.potential_threats_imgs.header = msg.header

				i = 0
				for person_box in self.potential_threats:
					# print("person_box[{}], xmin:[{}], xmax:[{}], ymin:[{}], ymax:[{}] ").format(i, person_box["xmin"], person_box["xmax"], person_box["ymin"], person_box["ymax"]) 
					crop_img = self.cv_image[person_box["ymin"]:person_box["ymax"],  person_box["xmin"]:person_box["xmax"]]
					self.potential_threats_imgs.images.append(self.bridge.cv2_to_imgmsg(crop_img, "bgr8"))
					i += 1

				self.threat_img_pub.publish(self.potential_threats_imgs)

		def box_overlap(self, human_box, gun_box):
				# Overlapping rectangles overlap both horizontally & vertically
				x_bool = self.range_overlap(human_box["xmin"], human_box["xmax"], gun_box["xmin"], gun_box["xmax"])
				y_bool = self.range_overlap(human_box["ymin"], human_box["ymax"], gun_box["ymin"], gun_box["ymax"])
				return x_bool and y_bool

		def range_overlap(self, a_min, a_max, b_min, b_max):
				# Neither range is completely greater than the other
				return (a_min <= b_max) and (b_min <= a_max)

if __name__ == '__main__':
		# Initialize the node and name it.
		rospy.init_node('evaluator')

		try:
				BoxEvaluator = box_evaluator()
		except rospy.ROSInterruptException: pass

		rate = rospy.Rate(30) # 30hz
		while not rospy.is_shutdown():
				rate.sleep()
