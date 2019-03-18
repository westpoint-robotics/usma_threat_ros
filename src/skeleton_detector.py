#!/usr/bin/env python

# cd ~/CMU/openpose/ && ./build/examples/openpose/openpose.bin --video examples/media/video.avi


# Import required Python code.
import math
# import sys
import sys; print sys.path
import rospy
import cv2
import numpy as np
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
        self.threat_boxes_topic = rospy.get_param("~threat_image_topic","/threats/potential_images")
        self.threat_boxes_sub = rospy.Subscriber(self.threat_boxes_topic,ImageArray,self.threat_boxes)
        self.msg_seq = 0
        self.msg_time = 0
        self.skeletons = []

        self.protoFile = rospy.get_param("~protoFile","cfg/coco/pose_deploy_linevec.prototxt")
        self.weightsFile = rospy.get_param("~weightsFile","cfg/coco/pose_iter_440000.caffemodel")
        self.MODE = rospy.get_param("~mode","COCO")

        if self.MODE is "COCO":
            self.nPoints = 18
            self.POSE_PAIRS = [ [1,0],[1,2],[1,5],[2,3],[3,4],[5,6],[6,7],[1,8],[8,9],[9,10],[1,11],[11,12],[12,13],[0,14],[0,15],[14,16],[15,17]]
        elif self.MODE is "MPI" :
            self.nPoints = 15
            self.POSE_PAIRS = [[0,1], [1,2], [2,3], [3,4], [1,5], [5,6], [6,7], [1,14], [14,8], [8,9], [9,10], [14,11], [11,12], [12,13] ]

        self.net = cv2.dnn.readNetFromCaffe(self.protoFile, self.weightsFile)
        self.blob_size = (51, 51)
        # self.blob_size = (368, 368)
        self.rbg_mean = (0, 0, 0)


        self.bridge = CvBridge()

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

                skeleton, keypoint_img, skel_img = self.calc_skeleton(image)
                print("skeleton computed")
                # self.skeletons.append(skeleton)
            
            cv2.imshow(('Output-Skeleton {}').format(i), skel_img)
            cv2.imshow(('Output-Keypoints {}').format(i), keypoint_img)
            cv2.waitKey(1)
            i+=1

            #do something with skeleton lists
            


    def calc_skeleton(self, img):
        frameCopy = np.copy(img)
        frameWidth = img.shape[1]
        frameHeight = img.shape[0]
        threshold = 0.1

        print("calc:skeleton:: (height x width) = ({}, {})").format(frameHeight, frameWidth)
        inpBlob = cv2.dnn.blobFromImage(img, 1.0 / 255, self.blob_size, self.rbg_mean, swapRB=False, crop=False)
        self.net.setInput(inpBlob)
        output = self.net.forward()

        H = output.shape[2]
        W = output.shape[3]

        # Empty list to store the detected keypoints
        points = []

        for i in range(self.nPoints):
            # confidence map of corresponding body's part.
            probMap = output[0, i, :, :]

            # Find global maxima of the probMap.
            minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
            
            # Scale the point to fit on the original image
            x = (frameWidth * point[0]) / W
            y = (frameHeight * point[1]) / H

            if prob > threshold : 
                cv2.circle(frameCopy, (int(x), int(y)), 8, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                cv2.putText(frameCopy, "{}".format(i), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, lineType=cv2.LINE_AA)

                # Add the point to the list if the probability is greater than the threshold
                points.append((int(x), int(y)))
            else :
                points.append(None)


        # Draw Skeleton
        for pair in self.POSE_PAIRS:
            partA = pair[0]
            partB = pair[1]

            if points[partA] and points[partB]:
                cv2.line(img, points[partA], points[partB], (0, 255, 255), 2)
                cv2.circle(img, points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)

        return points, point_img, skel_img




if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('skeleton_detector')

    try:
        skeleton = skeleton_detector()
    except rospy.ROSInterruptException: pass

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        rate.sleep()
