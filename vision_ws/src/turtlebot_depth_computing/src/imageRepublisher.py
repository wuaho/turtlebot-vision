#!/usr/bin/env python
# encoding: utf-8


import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_half_res",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/',Image,self.callback)
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if(cols > 60 and rows > 60) :
            cv2.circle(cv_image, (50,50), 10, 255)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
        except CvBridgeError as e:
            print(e)

    def main(args):
        ic = image_converter()
        rospy.init_node("image_converter")
        try:
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down")
        
    if __name__ == '__main__':
        main(sys.argv)