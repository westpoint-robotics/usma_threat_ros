#!/usr/bin/env python

# Import required Python code.
import math
import sys
import rospy
import cv2
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
import time
import tf

class box_evaluator():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # matlab config
        self.yolo_boxes_topic = rospy.get_param("~yolo_boxes_topic","/darknet_ros/bounding_boxes")
        self.yolo_boxes_sub = rospy.Subscriber(self.yolo_boxes_topic,BoundingBoxes,self.yolo_boxes)
        self.yolo_boxes_list = []
        self.msg_seq = 0
        self.msg_time = 0

    def yolo_boxes(self,msg):
        if (self.msg_time != msg.header.stamp.to_sec()):
            self.msg_time = msg.header.stamp.to_sec()
            self.msg_seq += 1
            self.yolo_box_data = msg.bounding_boxes
            # print("bounding_boxes: {} ").format(self.yolo_box_data) 
            print("\n")
            for box in self.yolo_box_data:
                print("box class: {} ").format(box.Class) 
                # if class == person, add to human list
                # if class == gun, add to gun list




if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('evaluator')

    try:
        BoxEvaluator = box_evaluator()
    except rospy.ROSInterruptException: pass

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        rate.sleep()



# ---
# header: 
#   seq: 272
#   stamp: 
#     secs: 1551282971
#     nsecs: 767909442
#   frame_id: "detection"
# image_header: 
#   seq: 1902
#   stamp: 
#     secs: 1551282971
#     nsecs: 477133782
#   frame_id: "ardrone_autonomy/ardrone_base_frontcam"
# bounding_boxes: 
#   - 
#     Class: "person"
#     probability: 0.912685096264
#     xmin: 335
#     ymin: 126
#     xmax: 608
#     ymax: 355
#   - 
#     Class: "cell phone"
#     probability: 0.954342365265
#     xmin: 310
#     ymin: 27
#     xmax: 490
#     ymax: 268
# ---




# struct rect
# {
#     int x;
#     int y;
#     int width;
#     int height;
# };

# bool valueInRange(int value, int min, int max)
# { return (value >= min) && (value <= max); }

# bool rectOverlap(rect A, rect B)
# {
#     bool xOverlap = valueInRange(A.x, B.x, B.x + B.width) ||
#                     valueInRange(B.x, A.x, A.x + A.width);

#     bool yOverlap = valueInRange(A.y, B.y, B.y + B.height) ||
#                     valueInRange(B.y, A.y, A.y + A.height);

#     return xOverlap && yOverlap;
# }

