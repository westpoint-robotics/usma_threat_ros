#!/usr/bin/env python

# cd ~/CMU/openpose/ && ./build/examples/openpose/openpose.bin --video examples/media/video.avi

# Import required Python code.
import math
import sys
import rospy
import cv2
from usma_threat_ros.msg import ImageArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import tf

class skeleton_detector():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # matlab config
        self.threat_boxes_topic = rospy.get_param("~threat_image_topic","/threats/potential images")
        self.threat_boxes_sub = rospy.Subscriber(self.threat_boxes_topic,ImageArray,self.threat_boxes)
        self.yolo_boxes_list = []
        self.msg_seq = 0
        self.msg_time = 0

        self.coco_protofile = rospy.get_param("~coco_protofile","cfg/coco/pose_deploy_linevec.prototxt")
        self.mpi_protofile = rospy.get_param("~coco_protofile","cfg/mpi/pose_deploy_linevec_faster_4_stages.prototxt")

        self.potential_threats_imgs = ImageArray()

        self.bridge = CvBridge()
        self.srcimg_topic = rospy.get_param("~srcimg_topic","/ardrone/front/image_rect_color")
        self.srcimg_sub = rospy.Subscriber(self.srcimg_topic,Image,self.updateimage)

    def threat_boxes(self,msg):
        if (self.msg_time != msg.header.stamp.to_sec()):
            self.msg_time = msg.header.stamp.to_sec()
            self.msg_seq += 1
            self.yolo_box_data = msg.bounding_boxes


        # try:
        #     self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #     self.image_rx = True
        # except CvBridgeError as e:
        #     print(e)


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
                print("person_box[{}], xmin:[{}], xmax:[{}], ymin:[{}], ymax:[{}] ").format(i, person_box["xmin"], person_box["xmax"], person_box["ymin"], person_box["ymax"]) 
                crop_img = self.cv_image[person_box["ymin"]:person_box["ymax"],  person_box["xmin"]:person_box["xmax"]]
                self.potential_threats_imgs.images.append(self.bridge.cv2_to_imgmsg(crop_img, "bgr8"))
                i += 1


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('skeleton_detector')

    try:
        skeleton = skeleton_detector()
    except rospy.ROSInterruptException: pass

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        rate.sleep()
