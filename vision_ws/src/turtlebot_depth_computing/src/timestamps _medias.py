#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
#TODO AÑADIR EL IMPORT DE CSV Y EL DE NUMPY EN EL PAQUETE
import csv
import numpy as np


data_list = []
bridge = CvBridge()

class Seguidor:
    
    def __init__(self):
        self.media = 0
        self.contador = 0
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/', Image, self.image_callback)

    def image_callback(self, msg):
        
        print("He recibido una imagen!")
        camera_stamp = msg.header.stamp
        #en un solo float 
        camera_stamp_float = float(camera_stamp.secs + camera_stamp.nsecs *10**-9)
        
        img_seq = msg.header.seq

        #CALCULO DEL TAMAÑO DE LA IMAGEN
        #TODO MIRAR SI DE VERDAD CAMBIA CUANDO SE MUEVE LA CAMARA 
        n_rows = msg.height 
        row_length = msg.step
        img_size = n_rows * row_length   

        
        
        
        try:
            #Mediante bridge, transformamos la imagen de ROS en una que pueda utilizar OpenCV
            cv2_img = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')

        except CvBridgeError, e:
            #Codigo ejecutado al producirse un error
            print(e)

        else:
            #CODIGO A EJECUTAR DE UN PROGRAMA DE VISION 

            #RGB a HSV y filtro verde 
            filtrado_verde,mascara_verde = bgr_to_hsv(cv2_img)
            
        
        #Calculamos el timestamp actual para saber cuanto tiempo ha pasado
        system_stamp = rospy.get_rostime()
        system_stamp_float = rospy.get_time()

        cv2.imshow('filtro',mascara_verde)
        cv2.imshow('filtrado',filtrado_verde)

        cv2.imshow('original',cv2_img)
        cv2.waitKey(1)

        #Para ver si sigue el orden
        if(self.contador==0):
            diferencia = 0
            self.contador = img_seq
        else:
            diferencia = system_stamp_float - camera_stamp_float 
            self.contador += 1

        
        self.media += diferencia
        #Añadimos a nuestra estructura de datos todos los parametros recogidos
        data_list.append((str(img_seq), str(self.contador),str(camera_stamp.secs), str(camera_stamp.nsecs), 
        str(system_stamp.secs), str(system_stamp.nsecs), str(img_size), str(diferencia),str(self.media)))
    
def write_data():
    #Metodo que escribe los datos recogidos en un fichero csv
    #FORMATO: Numero de secuencia de la imagen, contador,stamp de la camara (s), en ns, stamp del sistema, en ns, tamaño de la imagen

    with open('datos_recogidos.csv','w') as csvfile:
        csvfile.write("img_seq,counter,camera_stamp_secs,camera_stamp_nsecs,system_stamp_secs,system_stamp_nsecs,img_size")
        for stamp in data_list:
            line = ",".join(stamp)
            csvfile.write("\n")
            csvfile.write(line)

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
    #Nuestro nodo, seguidor, es iniciado
    rospy.init_node('seguidor')
    seguidor=Seguidor()
    #La siguiente linea sirve unicamente para cuando vaya a finalizar el nodo
    rospy.spin()
    #Escribimos todos los datos almacenados en el archivo
    write_data()
    
if __name__ == "__main__":
    main()
    


