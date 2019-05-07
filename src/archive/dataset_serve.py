#!/usr/bin/env python

# Import required Python code.
import math
import sys
import rospy
import cv2
from usma_threat_ros.msg import ImageArray
from usma_threat_ros.srv import DataSetLoad, DataSetImage
from os import walk # for listing contents of a directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class serve_dataset():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# ros config
		self.serve_load_topic = rospy.get_param("~dataset_load_topic","/usma_threat/load_dataset")
		self.serve_load_dataset = rospy.Service(self.serve_load_topic, DataSetLoad, self.request_load)
		self.dataset_folder = ""
		self.image_list = []

		self.request_image_topic = rospy.get_param("~request_image_topic","/usma_threat/request_image")
		self.serve_dataset_image = rospy.Service(self.request_image_topic, DataSetImage, self.request_serve)

		self.bridge = CvBridge()
		self.dataset_img_pub_topic = rospy.get_param("~dataset_img_pub_topic","/usma_threat/served_image")
		self.dataset_img_pub = rospy.Publisher(self.dataset_img_pub_topic, Image, queue_size=10)

	def request_load(self,req):
		self.image_list = []
		self.dataset_folder = str(req.path.data)
		print("Loading {}").format(self.dataset_folder)

		for (dirpath, dirnames, filenames) in walk(self.dataset_folder):
			self.image_list.extend(filenames)
			break
		return len(self.image_list)

	def request_serve(self,req):
		self.image_id = req.id
		current_image = self.dataset_folder+self.image_list[self.image_id]
		print("serving Image {}").format(current_image)
		image_to_publish = cv2.imread(current_image)
		try:
			# self.dataset_img_pub.publish(self.bridge.cv2_to_imgmsg(image_to_publish, "bgr8"))
			image_msg = self.bridge.cv2_to_imgmsg(image_to_publish, "bgr8")
			image_msg.header.seq = self.image_id
			image_msg.header.frame_id = "dataset"
			print("cv_image.header.seq: {}").format(image_msg.header.seq)
			# print("cv_image.height: {}").format(image_msg.height)
			# print("cv_image.width: {}").format(image_msg.width)
			# print("cv_image.encoding: {}").format(image_msg.encoding)
			self.dataset_img_pub.publish(image_msg)
			return True
		except CvBridgeError as e:
			print(e)
			return False

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('serve_dataset')

	try:
		ServeDataset = serve_dataset()
	except rospy.ROSInterruptException: pass

	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		rate.sleep()


# # From Python
# # It requires OpenCV installed for Python
# import sys; sys.path.append('/usr/local/python')
# import cv2
# from os import walk # for listing contents of a directory
# import argparse
# import yaml
# import numpy as np
# from openpose import pyopenpose as op


# def main():
#     with open("cmu_config.yml", 'r') as ymlfile:
#         if sys.version_info[0] > 2:
#             cfg = yaml.load(ymlfile, Loader=yaml.FullLoader)
#         else:
#             cfg = yaml.load(ymlfile)

#     # Custom Params (refer to include/openpose/flags.hpp for more parameters)
#     params = dict()
#     # params["model_folder"] = "/home/benjamin/CMU/openpose/models/"
#     params["model_folder"] = cfg['model_folder']
#     params["model_pose"] = cfg['model_pose']

#     # Starting OpenPose
#     opWrapper = op.WrapperPython()
#     opWrapper.configure(params)
#     opWrapper.start()

#     # Process Image
#     datum = op.Datum()

#     # walk 'unique' for renamable list
#     image_list = []
#     for (dirpath, dirnames, filenames) in walk(cfg['image_folder']):
#         image_list.extend(filenames)
#         break

#     images_processed = 0
#     for image in image_list:
#         current_image = cfg['image_folder']+image
#         keypoint_file = cfg['keypoint_folder']+image[:-4]
#         output_file = cfg['output_folder']+image
#         # print("output_file = {}".format(output_file))
#         # print("current_image = {}".format(current_image))

#         imageToProcess = cv2.imread(current_image)
#         datum.cvInputData = imageToProcess
#         opWrapper.emplaceAndPop([datum])

#         # Save numpy arrays
#         if cfg['save_keypoints']:
#             # print("Body keypoints: \n" + str(datum.poseKeypoints))
#             # print(type(datum.cvOutputData))
#             np.save(keypoint_file, datum.poseKeypoints)

#         # Display Image
#         if cfg['show_images']:
#             cv2.imshow(image, datum.cvOutputData)
#             cv2.waitKey(0)
#             cv2.destroyWindow(image)

#         if cfg['save_output_image']:
#             cv2.imwrite(output_file,datum.cvOutputData)

#         images_processed += 1
#         if images_processed%100 == 0:
#             print("\n    Images to process remaining in {} : {} \n").format(cfg['image_folder'], len(image_list)-images_processed)

# if __name__ == "__main__":
#     main()


# # 
