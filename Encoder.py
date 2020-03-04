#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 14:03:57 2020

@author: shane
"""
import rospy
import serial
import math
import numpy as np
#import time
from std_msgs.msg import Int32, Float32

rospy.init_node('Encoder', anonymous = False)
Encoder = rospy.Publisher('/Encoder_data', Int32, queue_size = 1)
rate = rospy.Rate(1)

global current_time
global previous_time

global first_encoder_read 
global second_encoder_read 


N_motor = 31 # gear ratio of motor
PPR = 20 # count per revolution
N_gear = 50.0/14 # gear rateio of drive train
Enc_speed_max = 3 # max round per second of encoder read
Enc_speed_ave_prev = 0 # previous speed 
speed_average_ratio = 50

first_encoder_read = 0
second_encoder_read = 0

def init_encoder_port(): # find encoder port address and initialize connection if found
    port = None
    for count in range(0,6):
        address = '/dev/ttyUSB' + str(count)
        print(count)
        for j in range(0,20):
            try:
                ser = serial.Serial(
                    port=address,
                    baudrate = 115200,
                    parity = 'N',
                    stopbits = 1,
                    bytesize = 8,
                    timeout = 0.05
                )
            
                ser.flushInput()
                rospy.sleep(0.1)
                read = ser.readline()
                #read = ser.readline()
                #print("data before chopping:"+read)
                position_read = str(read.strip())#.split('\\')[0][2:].replace('\U00002013','-')
                #print("data after chopping"+position_read)
                if len(position_read) >= 1 and position_read[1:].isdigit() or position_read.isdigit(): # identify whether a given port is connected to the encoder by checking the reading
                    port = ser
                    print('Encoder Port Found at ' + address)
                    return port
            except:
                pass
    print('Encoder Port Not Found')
    return None

def Encoder_Reading(port):
    while True:
        port.flushInput()
        Enc_read = port.readline()
        rate.sleep()
        Enc_read = str(Enc_read).strip()
        if len(Enc_read) >= 1 and Enc_read[1:].isdigit() or Enc_read.isdigit():
            number = int(Enc_read)
            print(number)
            Encoder.publish(number)
        else:
            continue
        

def Encoder_Speed_Calculator():
    


if __name__ == "__main__": 
    try: 
       Enc_ser = init_encoder_port()
       Encoder_Reading(Enc_ser)
    except rospy.ROSInterruptException:
        pass