#!/usr/bin/env python
# encoding: utf-8
import rospy
import time
import csv
from datetime import datetime


class ROS_Timer:    
    ###ATRIBUTOS DE LA CLASE QUE VAN A SER IGUALES EN CUALQUIER OBJETO CREADO DE DICHA CLASE
    #UN underscore es simplemente convencion para decir que algo es privado, un doble underscore hace  que te lo tengas que currar 
    #para acceder al valor

    def __init__(self):
        self.startTime = None
        self.stopTime = None
        
        self.firstImage = True
        self.imageCounter = 0
        self.frameSeqNumber = None
        self.expectedFrameSeqNumber = None
        
        
        self.cameraStamp = None
        self.receiptStamp = None
        self.postProcessingStamp = None

        
        self.lostPackets = 0
        self.totalLostPackets = 0
        self.sumImageSize = float(0)
        self.networkDelay = float(0)
        self.totalNetworkDelay = float(0)
        self.maxTime = float('-inf')
        self.minTime = float('inf')
    
        
        ###DATA STRUCTURE: LIST OF TUPLES. Each tuple: (frameSeqNumber,cameraStamp,receiptStamp,postProcessingStamp,
        # lostPackets, network delay,processing delay,total delay)
        self.storedData = []
    
    def __increaseCounter(self):
        self.imageCounter += 1
    
    def __addPacketsLost(self):
        """Sets the number of packets lost before the last one and sets the counter for the next expected image sequence number"""
        if self.expectedFrameSeqNumber is not None:
            self.lostPackets = self.frameSeqNumber - self.expectedFrameSeqNumber
            self.totalLostPackets += self.lostPackets

        self.expectedFrameSeqNumber = self.frameSeqNumber + 1
        
    def __calculateDelays(self):
        self.networkDelay = float(self.receiptStamp -self.cameraStamp)
        self.totalNetworkDelay += self.networkDelay

        if(self.networkDelay > self.maxTime):
            self.maxTime = self.networkDelay

        if(self.networkDelay < self.minTime and self.networkDelay >0):
            self.minTime = self.networkDelay

        processingDelay = float(self.postProcessingStamp - self.receiptStamp)
        totalDelay = float(self.postProcessingStamp - self.cameraStamp)

        return processingDelay,totalDelay

    def set_cameraStamp(self,timestamp):
        """Takes the timestamp when the robot took the frame and saves it as a float"""

        self.cameraStamp = float(timestamp.secs + timestamp.nsecs *10**-9)
    
    def set_receiptStamp(self):
        """Call this function after the frame was received byt the subscriber to set the receipt stamp"""
        self.receiptStamp = rospy.get_time()
    
    def set_postProcessingStamp(self):
        """Call this function after the frame was processed to set the post processing frame stamp"""
        self.postProcessingStamp = rospy.get_time()
    
    def set_frameSeqNumber(self,numberOfFrame):
        """Sets the sequence number for the frame took by the camera """
        self.frameSeqNumber = numberOfFrame
    
    def set_imageSize(self,array_size_in_bytes):
        """Sets the size of the image contained in the packet"""
        self.sumImageSize += float(array_size_in_bytes)
    
    def set_containedMessageInfo(self,msg):
        """Calls the functions set_cameraStamp and set_frameSeqNumber which save important information about when
        was the image taken by the camera and the sequence number of the frame"""
        self.set_cameraStamp(msg.header.stamp)
        self.set_frameSeqNumber(msg.header.seq)
        self.set_imageSize(len(msg.data))

    def get_totalReceivedImages(self):
        return self.imageCounter
    def get_totalLostImages(self):
        return self.totalLostPackets
    def get_percentageOfLostImages(self):
        suma = float(self.get_totalReceivedImages() + self.get_totalLostImages())
        pctg = self.get_totalLostImages() / suma *100
        
        return pctg
        
    def get_minTime(self):
        return self.minTime
    def get_meanTime(self):
        mean = self.totalNetworkDelay / self.get_totalReceivedImages()
        return mean

    def get_maxTime(self):
        return self.maxTime
    def get_runtime(self):
        """Returns the number of seconds since the timer was started to when it was stopped
        """
        return self.stopTime - self.startTime 
    def get_time_passed(self):
        """Returns the number of seconds since the timer was started to the actual time
        """
        return  time.time() - self.stopTime
    
    def get_receiptStamp(self):
        """Call this function after the frame was received by the subscriber to set the receipt stamp"""
        return self.receiptStamp
    
    def calculate_fps(self):
        """Returns the fps that has been achieved after stopping the timer"""
        fps = self.get_totalReceivedImages() / self.get_runtime()
        return fps
    def calculate_jitter(self):
        """Returns the jitter that has been achieved after stopping the timer. That is the mean difference of delay
         between consecutive packets"""
        diff = float(0)
        for i in range(len(self.storedData)-1):
            diff += abs(self.storedData[i][5] - self.storedData[i+1][5])

        jitter = diff / ( len(self.storedData) -1)

        return jitter
    
    def calculate_bw(self):
        """Returns the average bandwidth consumption in Mb/s"""
        sum_of_bytes = self.sumImageSize
        average_bytes_per_image = sum_of_bytes / self.get_totalReceivedImages()
        FPS = self.calculate_fps()
        bw = (((average_bytes_per_image / 1024) / 1024) * FPS)  * 8
        return bw
    
    def print_average_bytes_per_image(self):
        ###BORRAR MAS ADELANTE, ES SOLO UN METODO AUXILIAR
        sum_of_bytes = self.sumImageSize
        average_MB_bytes_per_image = sum_of_bytes / self.get_totalReceivedImages() /1024/1024
        print(average_MB_bytes_per_image,"MB")

       
    def start(self):
        """Starts the timer"""
        self.startTime = time.time()
    
    def stop(self):
        """Stops the timer"""
        self.stopTime = time.time()
    
    
    def add_frame_to_data(self):
        """Call this function before exiting finishing the code for each image """
        if(not(self.firstImage)):
            self.__increaseCounter()
            self.__addPacketsLost()
            
            processingDelay,totalDelay = self.__calculateDelays()


            self.storedData.append((self.frameSeqNumber,self.cameraStamp,self.receiptStamp, self.postProcessingStamp, 
                                    self.lostPackets,self.networkDelay,processingDelay,totalDelay))
        else:
            self.firstImage = False
        
    def write_data(self):
        """Writes a file in the same directory named "data_collected.csv" that contains all the appended information 
        """
        ###DATA STRUCTURE: LIST OF TUPLES. Each tuple: (frameSeqNumber,cameraStamp,receiptStamp,postProcessingStamp,
        # lostPackets, network delay,processing delay,total delay)

        with open('stats/data_collected.csv','w') as csvfile:
            csvfile.write("frameSeqNumber,cameraStamp,receiptStamp,postProcessingStamp,lostPackets,network delay,processing delay,total delay")
            for stamp in self.storedData:
                line = ",".join(map(str,stamp))
                csvfile.write("\n")
                csvfile.write(line)

    def write_usefulInfo(self):
        """Writes a file in the same directory named 'useful_info.txt' that contains some performance 
        information about the simulation"""
        with open('stats/useful_info.txt','w') as txtfile:
            txtfile.write("TOTAL TIME SPENT ON SIMULATION: "+str(self.get_runtime())+" s\n")
            txtfile.write("TOTAL NUMBER OF IMAGES RECEIVED: "+str(self.get_totalReceivedImages())+" images\n")
            txtfile.write("TOTAL NUMBER OF IMAGES LOST: "+str(self.get_totalLostImages())+" images\n")
            txtfile.write("% OF IMAGES LOST: "+str(self.get_percentageOfLostImages())+" %\n")
            txtfile.write("MEAN LATENCY ACHIEVED: "+ str(self.get_meanTime())+" s\n")
            txtfile.write("MAX LATENCY ACHIEVED: "+ str(self.get_maxTime())+" s\n")
            txtfile.write("MIN LATENCY ACHIEVED: "+ str(self.get_minTime())+" s\n")
            txtfile.write("AVERAGE JITTER: "+ str(self.calculate_jitter())+" s\n")
            txtfile.write("APROX FREQUENCY: "+ str(self.calculate_fps())+" FPS\n")
            txtfile.write("AVERAGE BANDWIDTH CONSUMPTION: " + str(self.calculate_bw()) +" Mb/s")



if __name__ == "__main__":
    print("Start")
    