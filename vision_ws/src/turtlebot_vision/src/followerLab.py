#!/usr/bin/env python
# encoding: utf-8

import rospy,cv2,csv
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np


class Follower:
    
    def __init__(self):
        #Raw image version
        #self.image_sub = rospy.Subscriber('camera/rgb/image_color/', Image, self.image_callback)

        #Compressed image version
        self.image_sub = rospy.Subscriber('camera/rgb/image_color/compressed', CompressedImage, self.image_callback)
        #Half res version
        #self.image_sub = rospy.Subscriber('image_half_res', Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=0)
        self.twist = Twist()
    def image_callback(self, msg):
        #Making use of bridge, we transform the msg to something that OpenCV can handle
        #image = CvBridge().imgmsg_to_cv2(msg,desired_encoding='passthrough')

        #For compressed images we would use this one:
        image = CvBridge().compressed_imgmsg_to_cv2(msg,desired_encoding='passthrough')
        

        #Vision processing code that we want to execute
        #RGB to HSV and green filter 
        filtered_with_yellow,yellow_mask = bgr_to_hsv(image)


        #Since we dont need the whole line on the picture for the robot, we will slice everything that is not in the 
        #20-row portion of the image. Doing this, we get the part that is one-meter distance from the robot.
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top + 20

        yellow_mask[0:search_top, 0:w] = 0
        yellow_mask[search_bot:h, 0:w] = 0

        #Here we calculate the centroid or center of mass of the remaining masked picture
        M = cv2.moments(yellow_mask)
        if( M['m00'] > 0 ):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)
            #Movement of the robot, first line calculates the error between the center column of the image and the center
            #of the line
            err = cx - w/2
            self.twist.linear.x = 0.08
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist) 


        cv2.imshow('filter',yellow_mask)
        cv2.imshow('filtered',filtered_with_yellow)
        cv2.imshow('original',image)
        cv2.waitKey(1)
            
    


def bgr_to_hsv(imagen):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)    
    # define range of yellow color in HSV
    lower_yellow = np.array([22, 93, 0])
    upper_yellow = np.array([45, 255, 255])     
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(imagen,imagen, mask= mask)
    
    return res,mask


def main():
    
    rospy.init_node('follower')
    follower=Follower()
    
    #Rospy.spin will detect when the node is stopped 
    rospy.spin()
    

if __name__ == "__main__":
    main()
    
    


