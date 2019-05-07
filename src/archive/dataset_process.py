#!/usr/bin/env python

# Import required Python code.
import math
import sys
import rospy
import cv2
from usma_threat_ros.msg import ImageArray
from usma_threat_ros.srv import DataSetLoad, DataSetImage
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from os import walk # for listing contents of a directory
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import time

class dataset_process():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		# ros config
		self.dataset_load_topic = rospy.get_param("~dataset_load_topic","/usma_threat/load_dataset")
		self.dataset_path=String() 
		self.dataset_path.data = rospy.get_param("~dataset_path","/home/benjamin/datasets/test/")

		rospy.wait_for_service(self.dataset_load_topic)
		self.dataset_load_call = rospy.ServiceProxy(self.dataset_load_topic, DataSetLoad)


		self.request_image_topic = rospy.get_param("~request_image_topic","/usma_threat/serve_dataset")
		rospy.wait_for_service(self.request_image_topic)
		self.dataset_image_call = rospy.ServiceProxy(self.request_image_topic, DataSetImage)

		#yolo params
		self.yolo_boxes_topic = rospy.get_param("~yolo_boxes_topic","/darknet_ros/bounding_boxes")
		self.yolo_boxes_sub = rospy.Subscriber(self.yolo_boxes_topic,BoundingBoxes,self.yolo_boxes)
		self.yolo_boxes_list = []
		self.msg_seq = 0
		self.msg_time = 0
		self.human_type = "person"
		self.gun_type = "pistol"


		try:
			path = self.dataset_path
			print("Asking to load {}").format(path)
			self.dataset_repsonse = self.dataset_load_call(path)
			print("Length of dataset = {}").format(self.dataset_repsonse.length)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


		# print("Asking to publish first image")
		# self.image_response = self.dataset_image_call(0)

		self.rate = rospy.Rate(0.5) # hz

		for id in range(0, self.dataset_repsonse.length):
			try:
				print("Asking to publish image {}").format(id)
				self.image_response = self.dataset_image_call(id)
				# print("Length of dataset = {}").format(self.dataset_length)
				self.rate.sleep()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
		
		rospy.signal_shutdown("end of dataset!")

	def yolo_boxes(self,msg):
		return True


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('dataset_process')

    try:
        DatasetProcess = dataset_process()
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
