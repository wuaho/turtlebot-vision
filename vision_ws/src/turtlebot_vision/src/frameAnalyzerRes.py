#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import numpy as np
from ROS_Timer import ROS_Timer

ros_timer = ROS_Timer()

class Camera_Frames_Analysis:
    
    def __init__(self):
        #Start of the timer
        ros_timer.start()
        self.image_sub = rospy.Subscriber('/image_half_res', Image, self.image_callback, queue_size=30)

    def image_callback(self, msg):
        #The first thing we do is to tell ros_timer to set the timestamp for the receipt time
        ros_timer.set_receiptStamp()
        print("Recibo imagen",msg.header.seq)
        
        try:
            #Making use of bridge, we transform the msg to something that OpenCV can handle
            cv2_img = CvBridge().imgmsg_to_cv2(msg,desired_encoding='passthrough')

        except CvBridgeError as e:
            #Code executed when an error is encountered
            print(e)

        else:
            #Vision processing code that we want to execute
            #RGB to HSV and green filter 
            filtered_with_green,green_mask = bgr_to_hsv(cv2_img)
        
        # We tell ros_timer to get the post processing stamp right after we manipulate the frame
        ros_timer.set_postProcessingStamp()
        
        cv2.imshow('filter',green_mask)
        cv2.imshow('filtered',filtered_with_green)
        cv2.imshow('original',cv2_img)
        cv2.waitKey(1)
        
        #Getting the timestamp stored in the message sent by the camera and make ros_timer handle it
        # and also getting the number sequence inside the image taken by the camera
        ros_timer.set_containedMessageInfo(msg)

        #We add the information we gathered to the data container
        ros_timer.add_frame_to_data()
        
        #Uncomment this line if you want to print the average size of the images taken
        ros_timer.print_average_bytes_per_image()
    


def bgr_to_hsv(imagen):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)    
    # define range of green color in HSV
    lower_green = np.array([36,0,0])
    upper_green = np.array([86,255,255])     
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(imagen,imagen, mask= mask)
    
    return res,mask


def main():
    
    #Our node, camera_frames_analysis, is started
    rospy.init_node('camera_frames_analysis')
    Analyzer=Camera_Frames_Analysis()
    
    #Rospy.spin will detect when the node is stopped 
    #rospy.spin()

    #If you want it to stop after a duration of time use this line instead
    rospy.sleep(300)

    #Stop of the timer
    ros_timer.stop()
    
    #We write the data stored in a csv file
    ros_timer.write_data()
    ros_timer.write_usefulInfo()
    

if __name__ == "__main__":
    main()
    
    


