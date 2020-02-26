#! /usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import serial
#import time
#import datetime
import math
#import numpy as np
#import os
#import RPi.GPIO as GPIO
from std_msgs.msg import Int32, Float32
rospy.init_node('imu_data', anonymous = False)
IMU = rospy.Publisher('/IMU_angle', Float32, queue_size = 5)
rate = rospy.Rate(0.5)

def conv_rad_to_deg_set(x,dec_place): # convert radian to degree
    return round(float(x)*180/3.1416,dec_place)

def init_IMU_port(): # find IMU port address and initialize serial connection if address is found
    port = None
    for count in range(0,6):
        address = '/dev/ttyACM' + str(count)
        for j in range(0,10):
            try:
                ser = serial.Serial(
                    port=address,
                    baudrate = 115200,
                    parity = 'N',
                    stopbits = 1,
                    bytesize = 8,
                    timeout = 0.1)
                #print(ser.is_open)
                
                rospy.sleep(0.1)
                #ser.flushInput()
                ser.write('\x3a\x38\x31\x5c\x6e') #get streaming slot \x3a\x38\x31\x5c\x6e    :81\n
                #ser.write(bytearray(':81\n','UTF-8'))
                rospy.sleep(0.1)
                IMU_read = ser.readline()
                #print(IMU_read)
                message = str(IMU_read).split(',')

                # check whether the IMU is found by checking the reading
                if len(message) == 8 and message[4].isdigit() and message[5].isdigit():
                    port = ser
                    print('IMU Port Found at ' + address)
                    return port
            except:
            
                pass
    print('IMU Port Not Found') 
    return None



def space_read(port,prev_read): # read serial from IMU
    try:
        print('This is the line read from IMU: '+ str(port.readline()))
        IMU_read = str(port.readline())[2:-5].split(',')
        port.flushInput()
        #print(len(IMU_read))
        #print(IMU_read)
        if len(IMU_read) == 9: # check if the reading is valid by checking the number of elements within the list
            return IMU_read
        else:
            #print("Read Error: " + str(IMU_read))
            return prev_read # if the new reading is not valid, use the previous reading
            
    except:
        #print("Read Error: " + str(IMU_read))
        return prev_read

def IMU_set(IMU_ser):
       
    IMU_ser.write(bytearray(':86\n','UTF-8')) #stop streaming   :86\n
    rospy.sleep(0.1)
    IMU_ser.flushInput() #flush buffer

    message_str = ':82,10000,4294967295,0\n' #streaming timing
    message_byte = bytearray(message_str,'UTF-8')
    IMU_ser.write(message_byte) #send out streaning timing 
    IMU_ser.write(bytearray(':83\n','UTF-8')) #get streaming timing
    rospy.sleep(0.1)
    IMU_read = IMU_ser.readline()
    IMU_ser.write(bytearray(':80,2,255,255,255,255,255,255,255\n','UTF-8')) #set streaming slot
    IMU_ser.write(bytearray(':81\n','UTF-8')) #get streaming slot
    rospy.sleep(0.1)
    
    IMU_ser.write(bytearray(':96\n','UTF-8')) #Tare with current orientation
    
    IMU_ser.write(bytearray(':85\n','UTF-8')) #start streaming
    
    return IMU_read

def IMU_shut_down(port):
    port.write(bytearray(':86\n','UTF-8'))
    port.close()
    
#IMU_read = space_read(IMU_ser,IMU_read)
def IMU_execute(IMU_ser,IMU_read):

    while(True):
        #print('111111111')
        rate.sleep()
        IMU_read = space_read(IMU_ser,IMU_read)
        #print('Raw IMU data: '+ IMU_read)
        angle = -math.atan2(float(IMU_read[7]),float(IMU_read[8])) # convert raw data to angle
        dat_deg = conv_rad_to_deg_set(angle,4)
        
        #print('Angle: '+ str(dat_deg))
        IMU.publish(dat_deg)
        print(dat_deg)

        if (dat_deg>60):
            IMU_shut_down(IMU_ser)
   

if __name__ == "__main__": 
    try: 
        IMU_func = init_IMU_port()
        First_read = IMU_set(IMU_func)
        IMU_execute(IMU_func,First_read)
    except rospy.ROSInterruptException:
        pass