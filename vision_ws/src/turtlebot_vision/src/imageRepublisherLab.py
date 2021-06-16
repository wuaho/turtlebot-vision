#!/usr/bin/env python
# encoding: utf-8


import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_half_res",Image,queue_size=0)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_color/',Image,self.callback)
    
    def callback(self,data):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        #We make the size a 1/4 of the original one
        cv_modified = cv2.resize(cv_image, (0,0), fx = 0.5, fy = 0.5)
        
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_modified,"bgr8")
            #The stamp inside the header is not saved after converting the image so we assign the same one that 
            #original message had
            
            ros_image.header.stamp = data.header.stamp
            print("Envio imagen",data.header.seq)
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node("image_converter")
    ic = image_converter()
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
        
if __name__ == '__main__':
    main(sys.argv)