#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
class Seguidor:

    def __init__(self):
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/', Image, self.image_callback)

    def image_callback(self, msg):
        print("He recibido una imagen!")
        a = msg.header.stamp
        print(type(a))
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        except CvBridgeError, e:
            print(e)
        else:
            #cv2.imwrite('camera_image.jpeg', cv2_img)
            cv2.imshow('frame',cv2_img)
            cv2.waitKey(1)

        # cv2.imshow("vision_robot", image)
        # cv2.waitKey(3)


def main():
    #Nuestro nodo, seguidor, es iniciado
    rospy.init_node('seguidor')
    seguidor=Seguidor()
    #La siguiente linea sirve unicamente para cuando vaya a finalizar el nodo
    rospy.spin()

if __name__ == "__main__":
    main()


