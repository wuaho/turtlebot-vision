#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

lista= []
bridge = CvBridge()
class Seguidor:

    def __init__(self):
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/', Image, self.image_callback)

    def image_callback(self, msg):
        print("He recibido una imagen!")
        camera_stamp = msg.header.stamp

        #CALCULO DEL TAMAÑO DE LA IMAGEN
        #TODO MIRAR SI DE VERDAD CAMBIA CUANDO SE MUEVE LA CAMARA 
        n_rows = msg.height 
        row_length = msg.step
        img_size = n_rows * row_length
        print(row_length)
        print(n_rows)
        print(img_size)    
        
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        except CvBridgeError, e:
            ###CODIGO QUE SE EJECUTA SI SE PRODUCE UN ERROR
            print(e)
        else:
            system_stamp = rospy.get_rostime()
            ###CODIGO A EJECUTAR 
        
        ###PARA CALCULAR LA DIFERENCIA
        #sec_dif = system_stamp.secs-camera_stamp.secs
        #nano_dif = system_stamp.nsecs-camera_stamp.nsecs
        #print(camera_stamp)
        #print("La diferencia es de ", sec_dif ,"segundos y de ", nano_dif,"nanosegundos")
        
        lista.append((system_stamp.secs,system_stamp.nsecs,camera_stamp.secs,camera_stamp.nsecs))
        
            


def main():
    #Nuestro nodo, seguidor, es iniciado
    rospy.init_node('seguidor')
    seguidor=Seguidor()
    #La siguiente linea sirve unicamente para cuando vaya a finalizar el nodo
    rospy.spin()
    print(lista)

if __name__ == "__main__":
    main()
    


