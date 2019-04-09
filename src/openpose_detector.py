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

                imagename_i = "detected skeleton {}".format(i)

                cv2.namedWindow(imagename_i, cv2.WINDOW_NORMAL)
                cv2.imshow(imagename_i, self.datum.cvOutputData)
                cv2.resizeWindow(imagename_i, image.shape[0]*4,image.shape[0]*4)
                cv2.waitKey(25)
                i+=1

            #do something with skeleton lists
            


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('openpose_detector')

    try:
        skeleton = openpose_detector()
    except rospy.ROSInterruptException: pass

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        rate.sleep()
