#!/usr/bin/env python
import time
import csv
from datetime import datetime


class ROS_Timer:    
    ###ATRIBUTOS DE LA CLASE QUE VAN A SER IGUALES EN CUALQUIER OBJETO CREADO DE DICHA CLASE
    #UN underscore es simplemente convencion para decir que algo es privado, un doble underscore hace  que te lo tengas que currar 
    #para acceder al valor

    def __init__(self):
        self.__startTime = None
        self.__stopTime = None
        self.__times = 0
        self.__minimumTime = float('inf')

    def start(self):
        self.__startTime = time.time()
    
    def stop(self):
        self.__stopTime = time.time()
    
    def get_runtime(self):
        """Returns the number of seconds since the timer was started to when it was stopped
        """
        return self.__stopTime - self.__startTime 
    def get_time_passed(self):
        """Returns the number of seconds since the timer was started to the actual time
        """
        return self.__stopTime - time.time()
    
    def write_data(self):
        """Writes a file in the same directory named "data_collected.csv" that contains all the appended information 
        """
        #FORMATO: Numero de secuencia de la imagen, contador,stamp de la camara (s), en ns, stamp del sistema, en ns, tamaño de la imagen

        with open('data_collected.csv','w') as csvfile:
            csvfile.write("img_seq,counter,camera_stamp_secs,camera_stamp_nsecs,system_stamp_secs,system_stamp_nsecs,img_size")
            for stamp in data_list:
                line = ",".join(stamp)
                csvfile.write("\n")
                csvfile.write(line)




if __name__ == "__main__":
    my_timer = ROS_Timer()
    my_timer.start()
    my_timer.stop()
    print(my_timer.get_runtime())
    print(my_timer.get_time_passed())
    