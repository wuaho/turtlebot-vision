#!/usr/bin/env python

### ESTE CODIGO NO FUNCIONA YA QUE SE ABRE LA VENTANA DONDE SE DEBERIAN DE MOSTRAR LAS IMAGENES PERO INMEDIATAMENTE SE APAGA
### SIN MOSTRAR MENSAJE DE ERROR

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Seguidor:

    def __init__(self):
        self.bridge = CvBridge()
        cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        cv2.imshow("window", image)
        cv2.waitKey()

rospy.init_node('seguidor')
seguidor=Seguidor()
rospy.spin()

# def main():
#     #Nuestro nodo, seguidor, es iniciado
#     rospy.init_node('seguidor')
#     seguidor=Seguidor()
#     #La siguiente linea sirve unicamente para cuando vaya a finalizar el nodo
#     rospy.spin()

# if __name__ == "__main__":
#     main()


